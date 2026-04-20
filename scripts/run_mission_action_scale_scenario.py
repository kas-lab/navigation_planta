#!/usr/bin/env python3

"""Experiment C — Mission action scaling with optional no-adaptation mode."""

import argparse
import subprocess
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import unified_planning
from unified_planning.shortcuts import BoolType, Object

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

from navigation_planta.map_generator import MapGenerator
from navigation_planta.no_adaptation import MissionNoAdaptationMapGenerator
from navigation_planta.utils import NO_PLAN, plot_memory_boxplot, run_planner_with_metrics

OWL_FILES = {
    1: REPO_ROOT / 'owl' / 'navigation.owl',
    2: REPO_ROOT / 'owl' / 'experiment_ma_scalability' / 'navigation_ma_2.owl',
    3: REPO_ROOT / 'owl' / 'experiment_ma_scalability' / 'navigation_ma_3.owl',
    4: REPO_ROOT / 'owl' / 'experiment_ma_scalability' / 'navigation_ma_4.owl',
    5: REPO_ROOT / 'owl' / 'experiment_ma_scalability' / 'navigation_ma_5.owl',
}

ADAPTIVE_DOMAIN_FILES = {
    1: REPO_ROOT / 'pddl' / 'domain_sas.pddl',
    2: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_sas_ma_2.pddl',
    3: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_sas_ma_3.pddl',
    4: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_sas_ma_4.pddl',
    5: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_sas_ma_5.pddl',
}

BASELINE_DOMAIN_FILES = {
    1: REPO_ROOT / 'pddl' / 'domain_no_sas.pddl',
    2: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_no_sas_ma_2.pddl',
    3: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_no_sas_ma_3.pddl',
    4: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_no_sas_ma_4.pddl',
    5: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_no_sas_ma_5.pddl',
}

MODE_LABELS = {
    'adaptive': 'Adaptive',
    'no-adaptation': 'No adaptation',
}

M_VALUES = [1, 2, 3, 4, 5]

N_NODES = 100
NODES_SKIP = 0.1
UNCONNECTED_AMOUNT = 0.15
UNSAFE_AMOUNT = 0.25
DARK_AMOUNT = 0.25


class MissionMapGenerator(MapGenerator):
    """MapGenerator extended with secondary mission action types."""

    ACTION_NAMES = ['inspection', 'delivery', 'recharge', 'report']

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount, m):
        super().__init__(
            num_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
        assert 1 <= m <= 5, f'M must be in 1..5, got {m}'
        self.m = m
        self.active_actions = self.ACTION_NAMES[:m - 1]

    def generate_domain(self):
        super().generate_domain()
        for action_name in self.active_actions:
            setattr(
                self,
                f'{action_name}_waypoint_fluent',
                unified_planning.model.Fluent(
                    f'{action_name}_waypoint', BoolType(), wp=self.waypoint_type))
            setattr(
                self,
                f'{action_name}_done_fluent',
                unified_planning.model.Fluent(
                    f'{action_name}_done', BoolType()))

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)

        if not self.active_actions:
            return

        sorted_nodes = sorted(self.graph.nodes)
        n = len(sorted_nodes)
        waypoints = {
            node_id: Object(f'wp{node_id}', self.waypoint_type)
            for node_id in self.graph.nodes
        }

        for i, action_name in enumerate(self.active_actions):
            wp_fluent = getattr(self, f'{action_name}_waypoint_fluent')
            done_fluent = getattr(self, f'{action_name}_done_fluent')
            self.problem.add_fluent(done_fluent, default_initial_value=False)

            target_node = sorted_nodes[(i + 1) * n // self.m]
            self.problem.set_initial_value(wp_fluent(waypoints[target_node]), True)
            self.problem.add_goal(done_fluent())


def make_generator(mode: str, m: int) -> MapGenerator:
    if mode == 'no-adaptation':
        return MissionNoAdaptationMapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT, m=m)
    if m == 1:
        return MapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT)
    return MissionMapGenerator(
        N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT, m=m)


def run_adaptive_preprocessor(owl_file, domain_file, problem_file, run_folder):
    domain_out = run_folder / 'domain_created.pddl'
    problem_out = run_folder / 'problem_created.pddl'
    subprocess.run(
        [
            'OWLToPDDL.sh',
            f'--owl={owl_file}',
            '--tBox',
            f'--inDomain={domain_file}',
            f'--outDomain={domain_out}',
            '--aBox',
            f'--inProblem={problem_file}',
            f'--outProblem={problem_out}',
            '--replace-output',
            '--add-num-comparisons',
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return domain_out, problem_out


def run_single(folder: Path, mode: str, run_id: int, m: int, search: str = 'lazy_greedy([ff()], preferred=[ff()])'):
    """Run one experiment trial."""
    run_folder = folder / mode / f'ma{m}_run{run_id}'
    run_folder.mkdir(parents=True, exist_ok=True)

    mg = make_generator(mode, m)
    mg.generate_connected_grid_map()

    problem_file = run_folder / (
        'problem.pddl' if mode == 'adaptive' else 'problem_no_sas.pddl')
    mg.generate_domain_problem_files(save_problem=True, problem_filename=problem_file)
    mg.plot_graph(show_plot=False, save_file=True, filename=run_folder / 'map.png')

    if mode == 'adaptive':
        owl_file = OWL_FILES[m]
        domain_file = ADAPTIVE_DOMAIN_FILES[m]
        owltopddl_start = time.perf_counter()
        try:
            domain_for_planner, problem_for_planner = run_adaptive_preprocessor(
                owl_file,
                domain_file,
                problem_file,
                run_folder,
            )
        except subprocess.CalledProcessError as error:
            print(f'OWLToPDDL failed (mode={mode}, m={m}, run={run_id}): {error.stderr}')
            return run_single(folder, mode, run_id, m, search)
        owltopddl_time = time.perf_counter() - owltopddl_start
    else:
        domain_for_planner = BASELINE_DOMAIN_FILES[m]
        problem_for_planner = problem_file
        owltopddl_time = 0.0

    plan_file = run_folder / 'plan'
    planning_time, action_count, peak_memory = run_planner_with_metrics(
        plan_file,
        domain_for_planner,
        problem_for_planner,
        search,
    )

    print(
        f'mode={mode:<13} m={m} run={run_id:2d} '
        f'owltopddl={owltopddl_time:.3f}s planning={planning_time:.6f}s '
        f'actions={action_count} peak_memory={peak_memory:.2f}MB'
    )
    return mode, m, owltopddl_time, planning_time, action_count, peak_memory


def save_results(folder: Path, records):
    csv_path = folder / 'planning_times.csv'
    single_mode = len({mode for mode, _, _, _, _, _ in records}) == 1

    if single_mode:
        arr = np.array(
            [(m, owltopddl_time, planning_time, action_count, peak_memory)
             for _, m, owltopddl_time, planning_time, action_count, peak_memory in records],
            dtype=[('m', 'i4'), ('owltopddl_time', 'f8'),
                   ('planning_time', 'f8'), ('action_count', 'i4'), ('peak_memory', 'f8')],
        )
        np.savetxt(
            csv_path, arr, delimiter=',',
            header='m,owltopddl_time,planning_time,action_count,peak_memory',
            comments='', fmt='%d,%.18e,%.18e,%d,%f',
        )
        return arr

    arr = np.array(
        [(mode, m, owltopddl_time, planning_time, action_count, peak_memory)
         for mode, m, owltopddl_time, planning_time, action_count, peak_memory in records],
        dtype=[('mode', 'U20'), ('m', 'i4'), ('owltopddl_time', 'f8'),
               ('planning_time', 'f8'), ('action_count', 'i4'), ('peak_memory', 'f8')],
    )
    np.savetxt(
        csv_path, arr, delimiter=',',
        header='mode,m,owltopddl_time,planning_time,action_count,peak_memory',
        comments='', fmt='%s,%d,%.18e,%.18e,%d,%f',
    )
    return arr


def plot_results(folder: Path, records, modes):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    for mode in modes:
        mean_t, std_t, mean_ac, std_ac = [], [], [], []
        for m in M_VALUES:
            times = np.array([
                pt for rm, rv, _, pt, _, _ in records if rm == mode and rv == m
            ])
            mean_t.append(times.mean())
            std_t.append(times.std())
            counts = np.array([
                ac for rm, rv, _, _, ac, _ in records
                if rm == mode and rv == m and ac != NO_PLAN
            ], dtype=float)
            mean_ac.append(counts.mean() if counts.size else float('nan'))
            std_ac.append(counts.std() if counts.size else 0.0)

        mean_t, std_t = np.array(mean_t), np.array(std_t)
        mean_ac, std_ac = np.array(mean_ac), np.array(std_ac)

        ax1.plot(M_VALUES, mean_t, marker='o', linestyle='-', label=MODE_LABELS[mode])
        ax1.fill_between(M_VALUES, mean_t - std_t, mean_t + std_t, alpha=0.2)
        ax2.plot(M_VALUES, mean_ac, marker='o', linestyle='-', label=MODE_LABELS[mode])
        ax2.fill_between(M_VALUES, mean_ac - std_ac, mean_ac + std_ac, alpha=0.2)

    ax1.set_xlabel('Number of Mission Action Types (M)')
    ax1.set_ylabel('Mean Planning Time (seconds)')
    ax1.set_title('Planning Time vs. Number of Mission Action Types')
    ax1.set_xticks(M_VALUES)
    ax1.legend()
    ax1.grid(True)

    ax2.set_xlabel('Number of Mission Action Types (M)')
    ax2.set_ylabel('Mean Plan Length (actions)')
    ax2.set_ylim(bottom=0)
    ax2.set_title('Plan Length vs. Number of Mission Action Types')
    ax2.set_xticks(M_VALUES)
    ax2.legend()
    ax2.grid(True)

    plot_path = folder / 'planning_time_ma_scale.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def runner(n_runs: int, mode: str, search: str = 'lazy_greedy([ff()], preferred=[ff()])', out_dir: Path | None = None) -> Path:
    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder = out_dir if out_dir is not None else (REPO_ROOT / 'results' / 'scalability_ma' / date)
    folder.mkdir(parents=True, exist_ok=True)

    modes = ['adaptive', 'no-adaptation'] if mode == 'both' else [mode]
    records = []

    for current_mode in modes:
        print(f'\n--- {MODE_LABELS[current_mode]} ---')
        for m in M_VALUES:
            print(f'  M = {m}')
            for run_id in range(n_runs):
                records.append(run_single(folder, current_mode, run_id, m, search))

    folder_mode = folder / mode
    folder_mode.mkdir(parents=True, exist_ok=True)
    csv_path = folder_mode / 'planning_times.csv'
    save_results(folder_mode, records)
    print(f'\nResults saved to {csv_path}')
    plot_results(folder_mode, records, modes)
    plot_memory_boxplot(folder_mode, records, MODE_LABELS, filename='peak_memory_boxplot.png')
    return csv_path


def main():
    parser = argparse.ArgumentParser(
        description='Experiment C: MA scaling — vary number of mission action types M.')
    parser.add_argument(
        '--mode',
        choices=['adaptive', 'no-adaptation', 'both'],
        default='adaptive',
        help=(
            'Experiment variant to run. '
            '"adaptive" reproduces the current PLANTA flow, '
            '"no-adaptation" uses the no-adaptation mission domains directly, '
            'and "both" runs both variants for comparison.'
        ),
    )
    parser.add_argument(
        '--runs', type=int, default=10, metavar='N',
        help='Number of runs per M value (default: 10)')
    parser.add_argument(
        '--search', default='astar(blind())',
        help='Fast Downward search configuration string (default: astar blind)')
    args = parser.parse_args()

    print(f'MA scaling experiment: mode={args.mode}, M={M_VALUES}, runs={args.runs}, nodes={N_NODES}')
    runner(args.runs, args.mode, args.search)


if __name__ == '__main__':
    main()
