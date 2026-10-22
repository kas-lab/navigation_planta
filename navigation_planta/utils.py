from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

NO_PLAN = -1

ROBOT_SPEED = 0.68  # m/s

# Energy cost per unit distance, keyed by configuration name with the
# 'fd_'/'conf_' scheme prefix stripped (see config_key()) so the PDDL
# ('fd_amcl_kinect') and PRISM ('conf_amcl_kinect') naming schemes share
# one source of truth.
CONFIG_ENERGY = {
    "amcl_kinect": 17790.0,
    "amcl_lidar": 19790.0,
    "mprt_kinect": 18942.0,
    "mprt_lidar": 20942.0,
    "aruco": 16963.0,
    "aruco_headlamp": 26963.0,
}


def config_key(configuration_name: str) -> str:
    """Strip a 'fd_' or 'conf_' scheme prefix to get the canonical config name."""
    for prefix in ('fd_', 'conf_'):
        if configuration_name.startswith(prefix):
            return configuration_name[len(prefix):]
    return configuration_name


def unrounded_energy_cost(distance: float, configuration_energy: float) -> float:
    return distance * ROBOT_SPEED * configuration_energy * 0.01


def energy_cost(distance: float, configuration_energy: float) -> int:
    return int(unrounded_energy_cost(distance, configuration_energy))


@dataclass(frozen=True)
class ExperimentRecord:
    """Shared record shape for experiment results."""

    mode: str
    x_value: int | str
    planning_time: float
    action_count: int
    move_action_count: int
    reconfig_action_count: int
    peak_memory: float
    remaining_battery: int | None = None
    owltopddl_time: float | None = None
    comparable_action_count: int | None = None
    distance: float | None = None


def run_subprocess_with_memory(
        command: list,
        sample_interval: float = 0.05,
        *,
        log_file: Path | None = None,
        accepted_returncodes: tuple[int, ...] = (0,)) -> float:
    """Return estimated peak aggregate RSS of a command and descendants in MiB.

    Sum live processes' RSS each sample (50 ms by default), then retain the
    largest total. Already discovered children remain tracked if reparented.
    The experiment runner is excluded. Shared pages may be counted more than
    once; brief peaks and children that exit between samples can be missed.

    Capture stdout/stderr without pipes so verbose children cannot deadlock.
    If log_file is set, retain the complete output there; otherwise use a
    temporary file. Unexpected exits raise CalledProcessError with an output
    tail, allowing the experiment retry layer to handle execution failures.
    """
    from contextlib import ExitStack
    import tempfile
    import math
    import subprocess
    import time
    import psutil

    if not math.isfinite(sample_interval) or sample_interval <= 0:
        raise ValueError("sample_interval must be finite and positive")

    max_rss = 0
    with ExitStack() as stack:
        output = stack.enter_context(
            Path(log_file).open('w+b') if log_file is not None
            else tempfile.TemporaryFile())
        process = stack.enter_context(subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT))
        try:
            tracked = {psutil.Process(process.pid)}
        except psutil.NoSuchProcess:
            tracked = set()

        while True:
            returncode = process.poll()
            # Process objects retain creation-time identity, avoiding PID reuse.
            for proc in list(tracked):
                try:
                    if proc.is_running():
                        tracked.update(proc.children(recursive=True))
                except psutil.NoSuchProcess:
                    pass

            total_rss = 0
            for proc in list(tracked):
                try:
                    if not proc.is_running() or proc.status() == psutil.STATUS_ZOMBIE:
                        tracked.discard(proc)
                        continue
                    total_rss += proc.memory_info().rss
                except psutil.NoSuchProcess:
                    tracked.discard(proc)
            max_rss = max(max_rss, total_rss)
            if returncode is not None and not tracked:
                break
            time.sleep(sample_interval)

        if process.returncode not in accepted_returncodes:
            output.seek(0, 2)
            output.seek(max(0, output.tell() - 65536))
            tail = output.read().decode('utf-8', errors='replace')
            error = subprocess.CalledProcessError(
                process.returncode, command, output=tail, stderr=tail)
            if log_file is not None:
                error.add_note(f'Command output saved to {log_file}')
            raise error
    return max_rss / (1024 * 1024)

def run_planner_with_metrics(
        plan_file: Path,
        domain_file: Path,
        problem_file: Path,
        search: str) -> tuple[float, int, int, int, float]:
    """Return wall time, plan counts, and estimated peak aggregate RSS (MiB)."""
    import time

    command = [
        'fast-downward.py',
        '--plan-file', str(plan_file),
        str(domain_file),
        str(problem_file),
        '--search', search,
    ]
    # A retry or a repeated invocation must never consume a previous plan.
    for candidate in plan_file.parent.glob(plan_file.name + '.*'):
        if candidate.name[len(plan_file.name) + 1:].isdigit():
            candidate.unlink()
    plan_file.unlink(missing_ok=True)

    start_time = time.perf_counter()
    peak_memory = run_subprocess_with_memory(
        command,
        log_file=plan_file.parent / 'planner.log',
        # FD 11: provably unsolvable; 12: search completed without a solution.
        # Resource limits and all execution/input errors still raise.
        accepted_returncodes=(0, 11, 12))
    planning_time = time.perf_counter() - start_time
    action_count, move_action_count, reconfig_action_count = count_plan_actions(plan_file)
    return planning_time, action_count, move_action_count, reconfig_action_count, peak_memory


def save_experiment_records_csv(
        csv_path: Path,
        records: Sequence[ExperimentRecord],
        x_name: str,
        time_name: str = 'planning_time') -> None:
    """Serialize experiment records, omitting ``mode`` for single-mode runs."""
    import csv

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    single_mode = len({record.mode for record in records}) == 1
    has_remaining_battery = any(
        record.remaining_battery is not None for record in records)
    has_owltopddl = any(record.owltopddl_time is not None for record in records)
    has_comparable = any(record.comparable_action_count is not None for record in records)
    has_distance = any(record.distance is not None for record in records)

    fieldnames = []
    if not single_mode:
        fieldnames.append('mode')
    fieldnames.append(x_name)
    if has_owltopddl:
        fieldnames.append('owltopddl_time')
    fieldnames.extend([time_name, 'action_count', 'move_action', 'reconfig_action', 'peak_memory'])
    if has_comparable:
        fieldnames.append('comparable_action_count')
    if has_remaining_battery:
        fieldnames.append('remaining_battery')
    if has_distance:
        fieldnames.append('distance')

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
                record.move_action_count,
                record.reconfig_action_count,
                f'{record.peak_memory:f}',
            ])
            if has_comparable:
                row.append(
                    '' if record.comparable_action_count is None
                    else record.comparable_action_count)
            if has_remaining_battery:
                row.append(record.remaining_battery)
            if has_distance:
                row.append(
                    '' if record.distance is None else f'{record.distance:f}')
            writer.writerow(row)


def _resolve_fd_plan_file(plan_file: Path) -> Path | None:
    """Return the actual FD plan file path, or None if not found.

    FD writes plan.1, plan.2, … — the last file is the best plan.
    """
    candidates = sorted(plan_file.parent.glob(plan_file.name + '.*'))
    if candidates:
        return candidates[-1]
    return plan_file if plan_file.exists() else None


def count_plan_actions(plan_file: Path) -> tuple[int, int, int]:
    """Count actions in an FD plan file, ignoring comment/cost lines.

    Returns NO_PLAN (-1) if no plan file is found.
    """
    target = _resolve_fd_plan_file(plan_file)
    if target is None:
        return NO_PLAN, -1, -1
    lines = target.read_text().splitlines()
    action_count = 0
    move_count = 0
    reconfigure_count = 0
    for line in lines:
        if line.strip() and not line.startswith(';'):
            action_count += 1
            if line.startswith('(move'):
                move_count += 1
            if line.startswith('(reconfigure'):
                reconfigure_count += 1
    return action_count, move_count, reconfigure_count


def corridor_energy_cost(
        corridor_distance: float, distance_by_configuration: dict[str, float]) -> int:
    """Return the energy cost of one original corridor.

    Each configuration is charged for its share of the traveled segments,
    scaled to the original corridor length, and the total is truncated once.
    This matches PRISM's one-truncation-per-corridor convention, so a corridor
    traversed under a single configuration costs exactly
    ``energy_cost(corridor_distance, ...)`` and segmentation adds no bias.
    """
    traveled = sum(distance_by_configuration.values())
    return int(sum(
        unrounded_energy_cost(
            corridor_distance * (distance / traveled),
            CONFIG_ENERGY[config_key(configuration)])
        for configuration, distance in distance_by_configuration.items()))


def count_comparable_plan_actions_and_energy(
        graph, plan_file: Path, original_node_ids: set[int], initial_battery: int,
        *, segment_graph=None, calculate_energy: bool = True) -> tuple[int, int | None, float]:
    """Count plan actions in the PRISM-comparable space.

    Move actions whose destination waypoint is an intermediate (discretized)
    node are skipped; all other actions count as 1. This lets discretized-PDDL
    action counts be compared directly with PRISM counts on the original map.
    Energy is attributed per discretized segment (looked up in
    ``segment_graph``, defaulting to ``graph``) to the configuration active
    while traversing it, so reconfiguring mid-corridor is charged correctly;
    see ``corridor_energy_cost()`` for the rounding convention.
    With calculate_energy=False, return battery=None: the no-adaptation
    baseline has no architectural configuration from which to infer energy.
    """
    target = _resolve_fd_plan_file(plan_file)
    if target is None:
        return NO_PLAN, -1 if calculate_energy else None, -1
    if segment_graph is None:
        segment_graph = graph
    count = 0
    remaining_battery = initial_battery if calculate_energy else None
    total_distance = 0.0
    configuration = ''
    init_id = -1
    distance_by_configuration: dict[str, float] = {}
    for line in target.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith(';'):
            continue
        tokens = line.strip('()').split()

        if tokens[0] == 'reconfigure':
            configuration = tokens[-1]

        if tokens[0] == 'move':
            wp_args = [t for t in tokens[1:] if t.startswith('wp')]
            source_id = int(wp_args[0][2:])
            dest_id = int(wp_args[1][2:])  # second wp-arg is destination
            if source_id in original_node_ids:
                init_id = source_id
                distance_by_configuration = {}
            if calculate_energy:
                distance_by_configuration[configuration] = (
                    distance_by_configuration.get(configuration, 0.0)
                    + segment_graph[source_id][dest_id]['weight'])
            if dest_id not in original_node_ids:
                continue
            distance = graph[init_id][dest_id]['weight']
            if calculate_energy:
                remaining_battery -= corridor_energy_cost(distance, distance_by_configuration)
            total_distance += distance
        count += 1
    return count, remaining_battery, total_distance
