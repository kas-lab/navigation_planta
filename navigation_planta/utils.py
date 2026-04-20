from pathlib import Path

NO_PLAN = -1


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


def plot_memory_boxplot(
        folder: Path,
        records: list,
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
        mems = np.array([r[-1] for r in records if r[0] == mode], dtype=float)
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
        records: list,
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
        times = np.array([t for m, _, t, _, _ in records if m == mode])
        counts = np.array(
            [ac for m, _, _, ac, _ in records if m == mode and ac != NO_PLAN],
            dtype=float)
        mems = np.array([pm for m, _, _, _, pm in records if m == mode])
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
