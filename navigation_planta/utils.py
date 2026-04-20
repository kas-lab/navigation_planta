from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

NO_PLAN = -1


@dataclass(frozen=True)
class ExperimentRecord:
    """Shared record shape for experiment results."""

    mode: str
    x_value: int | str
    planning_time: float
    action_count: int
    peak_memory: float
    owltopddl_time: float | None = None


def run_subprocess_with_memory(command: list) -> float:
    """Run *command* and return peak RSS memory usage in MB."""
    import subprocess
    import time
    import psutil

    process = subprocess.Popen(
        command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)
    try:
        proc = psutil.Process(process.pid)
    except psutil.NoSuchProcess:
        raise RuntimeError("Failed to monitor process; it exited too quickly.")
    max_rss = 0
    while process.poll() is None:
        try:
            max_rss = max(max_rss, proc.memory_info().rss)
        except psutil.NoSuchProcess:
            break
        time.sleep(0.05)
    if process.returncode != 0:
        print(f"[ERROR] Command failed: {' '.join(str(a) for a in command)}")
    return max_rss / (1024 * 1024)


def run_planner_with_metrics(
        plan_file: Path,
        domain_file: Path,
        problem_file: Path,
        search: str) -> tuple[float, int, float]:
    """Run Fast Downward and return wall-clock time, plan length, and peak RSS."""
    import time

    command = [
        'fast-downward.py',
        '--plan-file', str(plan_file),
        str(domain_file),
        str(problem_file),
        '--search', search,
    ]
    start_time = time.perf_counter()
    peak_memory = run_subprocess_with_memory(command)
    planning_time = time.perf_counter() - start_time
    action_count = count_plan_actions(plan_file)
    return planning_time, action_count, peak_memory


def save_experiment_records_csv(
        csv_path: Path,
        records: Sequence[ExperimentRecord],
        x_name: str,
        time_name: str = 'planning_time') -> None:
    """Serialize experiment records, omitting ``mode`` for single-mode runs."""
    import csv

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    single_mode = len({record.mode for record in records}) == 1
    has_owltopddl = any(record.owltopddl_time is not None for record in records)

    fieldnames = []
    if not single_mode:
        fieldnames.append('mode')
    fieldnames.append(x_name)
    if has_owltopddl:
        fieldnames.append('owltopddl_time')
    fieldnames.extend([time_name, 'action_count', 'peak_memory'])

    with csv_path.open('w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(fieldnames)
        for record in records:
            row = []
            if not single_mode:
                row.append(record.mode)
            row.append(record.x_value)
            if has_owltopddl:
                row.append(
                    ''
                    if record.owltopddl_time is None
                    else f'{record.owltopddl_time:.18e}')
            row.extend([
                f'{record.planning_time:.18e}',
                record.action_count,
                f'{record.peak_memory:f}',
            ])
            writer.writerow(row)
def count_plan_actions(plan_file: Path) -> int:
    """Count actions in an FD plan file, ignoring comment/cost lines.

    FD writes plans to {plan_file}.1, .2, ... — take the last (best for
    satisficing search). Returns NO_PLAN (-1) if no plan file is found.
    """
    candidates = sorted(plan_file.parent.glob(plan_file.name + '.*'))
    target = candidates[-1] if candidates else (plan_file if plan_file.exists() else None)
    if target is None:
        return NO_PLAN
    lines = target.read_text().splitlines()
    return sum(1 for line in lines if line.strip() and not line.startswith(';'))
