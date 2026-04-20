from pathlib import Path

from navigation_planta import experiment_runner
from navigation_planta.utils import ExperimentRecord


def test_resolve_modes_handles_both() -> None:
    assert experiment_runner.resolve_modes("both") == [
        "adaptive",
        "no-adaptation",
    ]
    assert experiment_runner.resolve_modes("adaptive") == ["adaptive"]


def test_run_with_retries_retries_then_succeeds() -> None:
    attempts = {"count": 0}

    def flaky() -> ExperimentRecord:
        attempts["count"] += 1
        if attempts["count"] < 3:
            raise RuntimeError("temporary")
        return ExperimentRecord("adaptive", 1, 0.1, 2, 3.0)

    record = experiment_runner.run_with_retries(
        flaky,
        attempts=3,
        context="unit-test",
    )

    assert attempts["count"] == 3
    assert record.action_count == 2


def test_execute_pddl_case_records_owltopddl_when_requested(
    monkeypatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(
        experiment_runner,
        "run_planner_with_metrics",
        lambda plan_file, domain_file, problem_file, search: (0.75, 4, 12.5),
    )

    def prepare_problem(run_folder: Path, mode: str) -> Path:
        problem_file = run_folder / "problem.pddl"
        problem_file.write_text("(define (problem test))")
        return problem_file

    def adaptive_preprocessor(problem_file: Path, run_folder: Path) -> tuple[Path, Path]:
        domain_file = run_folder / "domain_created.pddl"
        problem_out = run_folder / "problem_created.pddl"
        domain_file.write_text("(define (domain test))")
        problem_out.write_text(problem_file.read_text())
        return domain_file, problem_out

    perf_counter_values = iter([10.0, 10.2])
    monkeypatch.setattr(
        "time.perf_counter",
        lambda: next(perf_counter_values),
    )

    record = experiment_runner.execute_pddl_case(
        folder=tmp_path,
        mode="adaptive",
        case_id="case0",
        x_value=5,
        search="astar(blind())",
        prepare_problem=prepare_problem,
        adaptive_preprocessor=adaptive_preprocessor,
        baseline_domain=tmp_path / "baseline.pddl",
        include_owltopddl_time=True,
    )

    assert record.mode == "adaptive"
    assert record.x_value == 5
    assert record.planning_time == 0.75
    assert record.action_count == 4
    assert record.peak_memory == 12.5
    assert abs(record.owltopddl_time - 0.2) < 1e-9


def test_run_sweep_experiment_writes_csv_and_uses_mode_folder(tmp_path: Path) -> None:
    plotted = {"called": False}
    finalized = {"called": False}

    config = experiment_runner.SweepExperimentConfig(
        results_root=tmp_path / "results",
        mode_labels={"adaptive": "Adaptive", "no-adaptation": "No adaptation"},
        x_values=[1, 2],
        x_name="nodes",
        time_name="time",
        per_mode_results_folder=True,
        retry_attempts=2,
    )

    def run_one(folder: Path, mode: str, run_id: int, x_value: int, search: str) -> ExperimentRecord:
        return ExperimentRecord(mode, x_value, planning_time=float(x_value), action_count=run_id + 1, peak_memory=10.0)

    def plot_results(folder: Path, records, modes) -> None:
        plotted["called"] = True
        (folder / "plot.txt").write_text(f"{len(records)}:{','.join(modes)}")

    def after_run(folder: Path, records, modes) -> None:
        finalized["called"] = True
        (folder / "after.txt").write_text(str(len(records)))

    csv_path = experiment_runner.run_sweep_experiment(
        config,
        n_runs=2,
        mode="adaptive",
        run_one=run_one,
        plot_results=plot_results,
        search="astar(blind())",
        out_dir=tmp_path / "out",
        after_run=after_run,
    )

    assert plotted["called"] is True
    assert finalized["called"] is True
    assert csv_path == tmp_path / "out" / "adaptive" / "planning_times.csv"
    assert csv_path.exists()
    assert (tmp_path / "out" / "adaptive" / "plot.txt").exists()
    assert (tmp_path / "out" / "adaptive" / "after.txt").exists()
