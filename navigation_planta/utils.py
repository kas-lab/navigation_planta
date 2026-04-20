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


def _record_mode(record: ExperimentRecord | tuple) -> str:
    return record.mode if isinstance(record, ExperimentRecord) else record[0]


def _record_x_value(record: ExperimentRecord | tuple) -> int | str:
    return record.x_value if isinstance(record, ExperimentRecord) else record[1]


def _record_planning_time(record: ExperimentRecord | tuple) -> float:
    return record.planning_time if isinstance(record, ExperimentRecord) else record[2]


def _record_action_count(record: ExperimentRecord | tuple) -> int:
    return record.action_count if isinstance(record, ExperimentRecord) else record[3]


def _record_peak_memory(record: ExperimentRecord | tuple) -> float:
    return record.peak_memory if isinstance(record, ExperimentRecord) else record[-1]


def _record_owltopddl_time(record: ExperimentRecord | tuple) -> float | None:
    if isinstance(record, ExperimentRecord):
        return record.owltopddl_time
    return record[2] if len(record) == 6 else None


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


def plot_memory_boxplot(
        folder: Path,
        records: Sequence[ExperimentRecord | tuple],
        mode_labels: dict,
        filename: str = 'peak_memory_boxplot.png') -> None:
    """Peak-memory summary plot by mode using the same mean/std style as Camara.

    records: list of tuples where record[0] is the mode key and record[-1] is
    peak_memory in MB.  mode_labels maps mode key → display label.
    """
    import matplotlib.pyplot as plt
    import numpy as np

    modes = list(mode_labels.keys())
    labels = []
    mean_mems, std_mems = [], []
    for mode in modes:
        mems = np.array([
            _record_peak_memory(record)
            for record in records
            if _record_mode(record) == mode
        ], dtype=float)
        if mems.size:
            labels.append(mode_labels[mode])
            mean_mems.append(mems.mean())
            std_mems.append(mems.std())
    if not mean_mems:
        return

    fig, ax = plt.subplots(figsize=(max(6, 3 * len(mean_mems)), 5))
    x = np.arange(len(mean_mems))
    ax.bar(
        x,
        mean_mems,
        yerr=std_mems,
        capsize=5,
        color='mediumseagreen',
        alpha=0.8,
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('Peak Memory (MB)')
    ax.set_title('Peak Memory by Mode')
    ax.set_ylim(bottom=0)
    ax.grid(True, axis='y')

    plot_path = folder / filename
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def plot_camara_results(
        folder: Path,
        records: Sequence[ExperimentRecord | tuple],
        mode_labels: dict,
        filename: str = 'camara_summary.png') -> None:
    """Bar chart (mean ± std) of time, plan length, and peak memory per mode.

    records: list of (mode, init_goal, time, action_count, peak_memory) tuples.
    mode_labels: maps mode key → display label, defines subplot order.
    """
    import matplotlib.pyplot as plt
    import numpy as np

    modes = list(mode_labels.keys())
    labels = [mode_labels[m] for m in modes]
    mean_times, std_times = [], []
    mean_actions, std_actions = [], []
    mean_mems, std_mems = [], []

    for mode in modes:
        times = np.array([
            _record_planning_time(record)
            for record in records
            if _record_mode(record) == mode
        ])
        counts = np.array(
            [
                _record_action_count(record)
                for record in records
                if _record_mode(record) == mode and _record_action_count(record) != NO_PLAN
            ],
            dtype=float)
        mems = np.array([
            _record_peak_memory(record)
            for record in records
            if _record_mode(record) == mode
        ])
        mean_times.append(times.mean())
        std_times.append(times.std())
        mean_actions.append(counts.mean() if counts.size else float('nan'))
        std_actions.append(counts.std() if counts.size else 0.0)
        mean_mems.append(mems.mean())
        std_mems.append(mems.std())

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))
    x = np.arange(len(modes))

    ax1.bar(x, mean_times, yerr=std_times, capsize=5, color='steelblue', alpha=0.8)
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels)
    ax1.set_ylabel('Mean Planning Time (seconds)')
    ax1.set_title('Planning Time by Mode')
    ax1.set_ylim(bottom=0)
    ax1.grid(True, axis='y')

    ax2.bar(x, mean_actions, yerr=std_actions, capsize=5, color='darkorange', alpha=0.8)
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels)
    ax2.set_ylabel('Mean Plan Length (actions)')
    ax2.set_title('Plan Length by Mode')
    ax2.set_ylim(bottom=0)
    ax2.grid(True, axis='y')

    ax3.bar(x, mean_mems, yerr=std_mems, capsize=5, color='mediumseagreen', alpha=0.8)
    ax3.set_xticks(x)
    ax3.set_xticklabels(labels)
    ax3.set_ylabel('Mean Peak Memory (MB)')
    ax3.set_title('Peak Memory by Mode')
    ax3.set_ylim(bottom=0)
    ax3.grid(True, axis='y')

    plot_path = folder / filename
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


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
