"""Shared orchestration helpers for navigation_planta experiment scripts."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Generic, Sequence, TypeVar

from .utils import ExperimentRecord, run_planner_with_metrics, save_experiment_records_csv

X = TypeVar('X')


@dataclass(frozen=True)
class SweepExperimentConfig(Generic[X]):
    """Configuration for a repeated experiment sweep."""

    results_root: Path
    mode_labels: dict[str, str]
    x_values: Sequence[X]
    x_name: str
    time_name: str = 'planning_time'
    per_mode_results_folder: bool = True
    retry_attempts: int = 3


def resolve_modes(mode: str) -> list[str]:
    return ['adaptive', 'no-adaptation'] if mode == 'both' else [mode]


def resolve_output_folder(results_root: Path, out_dir: Path | None) -> Path:
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        return out_dir

    folder = results_root / datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def run_with_retries(
        operation: Callable[[], ExperimentRecord],
        *,
        attempts: int,
        context: str) -> ExperimentRecord:
    """Retry a case execution a bounded number of times."""
    if attempts < 1:
        raise ValueError(f'attempts must be >= 1, got {attempts}')

    last_error: Exception | None = None
    for attempt in range(1, attempts + 1):
        try:
            return operation()
        except Exception as error:
            last_error = error
            if attempt >= attempts:
                break
            print(
                f'Case failed ({context}) on attempt {attempt}/{attempts}: {error!r}. '
                f'Retrying...')

    assert last_error is not None
    raise last_error


def execute_pddl_case(
        *,
        folder: Path,
        mode: str,
        case_id: str,
        x_value: int | str,
        search: str,
        prepare_problem: Callable[[Path, str], Path],
        adaptive_preprocessor: Callable[[Path, Path], tuple[Path, Path]],
        baseline_domain: Path | Callable[[], Path],
        include_owltopddl_time: bool = False) -> ExperimentRecord:
    """Run one PDDL planning case and return a shared result record."""
    import time

    run_folder = folder / mode / case_id
    run_folder.mkdir(parents=True, exist_ok=True)
    problem_file = prepare_problem(run_folder, mode)

    if mode == 'adaptive':
        owltopddl_start = time.perf_counter()
        domain_for_planner, problem_for_planner = adaptive_preprocessor(
            problem_file,
            run_folder,
        )
        owltopddl_time = time.perf_counter() - owltopddl_start
    else:
        domain_for_planner = baseline_domain() if callable(baseline_domain) else baseline_domain
        problem_for_planner = problem_file
        owltopddl_time = 0.0

    plan_file = run_folder / 'plan'
    planning_time, action_count, peak_memory = run_planner_with_metrics(
        plan_file,
        domain_for_planner,
        problem_for_planner,
        search,
    )
    return ExperimentRecord(
        mode=mode,
        x_value=x_value,
        planning_time=planning_time,
        action_count=action_count,
        peak_memory=peak_memory,
        owltopddl_time=owltopddl_time if include_owltopddl_time else None,
    )


def run_sweep_experiment(
        config: SweepExperimentConfig[X],
        *,
        n_runs: int,
        mode: str,
        run_one: Callable[[Path, str, int, X, str], ExperimentRecord],
        plot_results: Callable[[Path, Sequence[ExperimentRecord], Sequence[str]], None],
        search: str,
        out_dir: Path | None = None,
        after_run: Callable[[Path, Sequence[ExperimentRecord], Sequence[str]], None] | None = None,
        value_printer: Callable[[X], str | None] | None = None) -> Path:
    """Run an experiment sweep with shared folder and CSV handling."""
    folder = resolve_output_folder(config.results_root, out_dir)
    modes = resolve_modes(mode)
    records: list[ExperimentRecord] = []

    for current_mode in modes:
        print(f'\n--- {config.mode_labels[current_mode]} ---')
        for x_value in config.x_values:
            if value_printer is not None:
                value_message = value_printer(x_value)
                if value_message:
                    print(value_message)
            for run_id in range(n_runs):
                context = f'mode={current_mode} value={x_value} run={run_id}'
                record = run_with_retries(
                    lambda current_mode=current_mode, run_id=run_id, x_value=x_value: run_one(
                        folder,
                        current_mode,
                        run_id,
                        x_value,
                        search,
                    ),
                    attempts=config.retry_attempts,
                    context=context,
                )
                records.append(record)

    results_folder = folder / mode if config.per_mode_results_folder else folder
    results_folder.mkdir(parents=True, exist_ok=True)
    csv_path = results_folder / 'planning_times.csv'
    save_experiment_records_csv(
        csv_path,
        records,
        x_name=config.x_name,
        time_name=config.time_name,
    )
    print(f'\nResults saved to {csv_path}')
    plot_results(results_folder, records, modes)
    if after_run is not None:
        after_run(results_folder, records, modes)
    return csv_path
