#!/usr/bin/env python3

"""Experiment D — Combined stress test with optional no-adaptation mode."""

import argparse
import random
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
from navigation_planta.utils import count_plan_actions, NO_PLAN

OWL_FILE = REPO_ROOT / 'owl' / 'experiment_combined_scalability' / 'navigation_combined.owl'
ADAPTIVE_DOMAIN_FILE = (
    REPO_ROOT / 'pddl' / 'experiment_combined_scalability' / 'domain_sas_combined.pddl'
)
BASELINE_DOMAIN_FILE = (
    REPO_ROOT / 'pddl' / 'experiment_combined_scalability' / 'domain_no_sas_combined.pddl'
)

MODE_LABELS = {
    'adaptive': 'Adaptive',
    'no-adaptation': 'No adaptation',
}

MIN_NODES = 10
MAX_NODES = 100
NODES_STEP = 10
NODES_SKIP = 0.1
UNCONNECTED_AMOUNT = 0.15
UNSAFE_AMOUNT = 0.25
DARK_AMOUNT = 0.25
OUTDOOR_AMOUNT = 0.20
DUSTY_AMOUNT = 0.20


class CombinedMapGenerator(MapGenerator):
    """MapGenerator at maximum adaptive complexity: K=4 and M=5."""

    ACTION_NAMES = ['inspection', 'delivery', 'recharge', 'report']
    M = 5

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount,
                 outdoor_amount=OUTDOOR_AMOUNT, dusty_amount=DUSTY_AMOUNT):
        super().__init__(
            num_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
        self.outdoor_amount = outdoor_amount
        self.dusty_amount = dusty_amount

    def assign_dark_unsafe_corridors(self, graph):
        for u, v in graph.edges:
            graph[u][v]['outdoor'] = False
            graph[u][v]['dusty'] = False

        graph = super().assign_dark_unsafe_corridors(graph)

        remaining_edges = list(graph.edges)
        if self.outdoor_amount > 0:
            num_outdoor = int(self.outdoor_amount * len(remaining_edges))
            for u, v in random.sample(remaining_edges, num_outdoor):
                graph[u][v]['outdoor'] = True
        if self.dusty_amount > 0:
            num_dusty = int(self.dusty_amount * len(remaining_edges))
            for u, v in random.sample(remaining_edges, num_dusty):
                graph[u][v]['dusty'] = True

        return graph

    def generate_domain(self):
        super().generate_domain()
        self.outdoor_requirement = unified_planning.model.Fluent(
            'outdoor_requirement', BoolType(),
            wp1=self.waypoint_type, wp2=self.waypoint_type,
            v=self.numerical_object_type)
        self.dust_requirement = unified_planning.model.Fluent(
            'dust_requirement', BoolType(),
            wp1=self.waypoint_type, wp2=self.waypoint_type,
            v=self.numerical_object_type)
        for action_name in self.ACTION_NAMES:
            setattr(
                self,
                f'{action_name}_waypoint_fluent',
                unified_planning.model.Fluent(
                    f'{action_name}_waypoint', BoolType(), wp=self.waypoint_type))
            setattr(
                self,
                f'{action_name}_done_fluent',
                unified_planning.model.Fluent(f'{action_name}_done', BoolType()))

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)

        waypoints = {
            node_id: Object(f'wp{node_id}', self.waypoint_type)
            for node_id in self.graph.nodes
        }
        zero_decimal = Object('0.0_decimal', self.numerical_object_type)
        zero_eight_decimal = Object('0.8_decimal', self.numerical_object_type)

        for u, v in self.graph.edges:
            req_out = zero_eight_decimal if self.graph[u][v].get('outdoor') else zero_decimal
            self.problem.set_initial_value(
                self.outdoor_requirement(waypoints[u], waypoints[v], req_out), True)
            self.problem.set_initial_value(
                self.outdoor_requirement(waypoints[v], waypoints[u], req_out), True)

            req_dust = zero_eight_decimal if self.graph[u][v].get('dusty') else zero_decimal
            self.problem.set_initial_value(
                self.dust_requirement(waypoints[u], waypoints[v], req_dust), True)
            self.problem.set_initial_value(
                self.dust_requirement(waypoints[v], waypoints[u], req_dust), True)

        sorted_nodes = sorted(self.graph.nodes)
        n = len(sorted_nodes)

        for i, action_name in enumerate(self.ACTION_NAMES):
            wp_fluent = getattr(self, f'{action_name}_waypoint_fluent')
            done_fluent = getattr(self, f'{action_name}_done_fluent')
            self.problem.add_fluent(done_fluent, default_initial_value=False)

            target_node = sorted_nodes[(i + 1) * n // self.M]
            self.problem.set_initial_value(wp_fluent(waypoints[target_node]), True)
            self.problem.add_goal(done_fluent())


def make_generator(mode: str, n_nodes: int) -> MapGenerator:
    if mode == 'no-adaptation':
        return MissionNoAdaptationMapGenerator(
            n_nodes, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT, m=5)
    return CombinedMapGenerator(
        n_nodes, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT)


def run_adaptive_preprocessor(problem_file, run_folder):
    domain_out = run_folder / 'domain_created.pddl'
    problem_out = run_folder / 'problem_created.pddl'
    subprocess.run(
        [
            'OWLToPDDL.sh',
            f'--owl={OWL_FILE}',
            '--tBox',
            f'--inDomain={ADAPTIVE_DOMAIN_FILE}',
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


def run_single(folder: Path, mode: str, run_id: int, n_nodes: int, search: str = 'astar(blind())'):
    """Run one trial for a given map size."""
    n_nodes_resulting = n_nodes - int(n_nodes * NODES_SKIP)
    run_folder = folder / mode / f'n{n_nodes}_run{run_id}'
    run_folder.mkdir(parents=True, exist_ok=True)

    mg = make_generator(mode, n_nodes)
    mg.generate_connected_grid_map()

    problem_file = run_folder / (
        'problem.pddl' if mode == 'adaptive' else 'problem_no_sas.pddl')
    mg.generate_domain_problem_files(save_problem=True, problem_filename=problem_file)
    mg.plot_graph(show_plot=False, save_file=True, filename=run_folder / 'map.png')

    if mode == 'adaptive':
        owltopddl_start = time.perf_counter()
        try:
            domain_for_planner, problem_for_planner = run_adaptive_preprocessor(
                problem_file,
                run_folder,
            )
        except subprocess.CalledProcessError as error:
            print(f'OWLToPDDL failed (mode={mode}, n={n_nodes}, run={run_id}): {error.stderr}')
            return run_single(folder, mode, run_id, n_nodes, search)
        owltopddl_time = time.perf_counter() - owltopddl_start
    else:
        domain_for_planner = BASELINE_DOMAIN_FILE
        problem_for_planner = problem_file
        owltopddl_time = 0.0

    plan_file = run_folder / 'plan'
    fd_start = time.perf_counter()
    subprocess.run(
        [
            'fast-downward.py',
            '--plan-file', str(plan_file),
            str(domain_for_planner),
            str(problem_for_planner),
            '--search', search,
        ],
        capture_output=True,
        check=True,
    )
    planning_time = time.perf_counter() - fd_start
    action_count = count_plan_actions(plan_file)

    print(
        f'mode={mode:<13} n={n_nodes:4d} (eff={n_nodes_resulting:3d}) run={run_id:2d} '
        f'owltopddl={owltopddl_time:.3f}s planning={planning_time:.6f}s actions={action_count}'
    )
    return mode, n_nodes_resulting, owltopddl_time, planning_time, action_count


def save_results(folder: Path, records):
    csv_path = folder / 'planning_times.csv'
    single_mode = len({mode for mode, _, _, _, _ in records}) == 1

    if single_mode:
        arr = np.array(
            [(nodes, owltopddl_time, planning_time, action_count)
             for _, nodes, owltopddl_time, planning_time, action_count in records],
            dtype=[('nodes', 'i4'), ('owltopddl_time', 'f8'),
                   ('planning_time', 'f8'), ('action_count', 'i4')],
        )
        np.savetxt(
            csv_path, arr, delimiter=',',
            header='nodes,owltopddl_time,planning_time,action_count',
            comments='', fmt='%d,%.18e,%.18e,%d',
        )
        return arr

    arr = np.array(
        [(mode, nodes, owltopddl_time, planning_time, action_count)
         for mode, nodes, owltopddl_time, planning_time, action_count in records],
        dtype=[('mode', 'U20'), ('nodes', 'i4'), ('owltopddl_time', 'f8'),
               ('planning_time', 'f8'), ('action_count', 'i4')],
    )
    np.savetxt(
        csv_path, arr, delimiter=',',
        header='mode,nodes,owltopddl_time,planning_time,action_count',
        comments='', fmt='%s,%d,%.18e,%.18e,%d',
    )
    return arr


def plot_results(folder: Path, records, modes):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5))

    unique_nodes = sorted({nodes for _, nodes, _, _, _ in records})
    for mode in modes:
        mean_t, std_t, mean_ac, std_ac = [], [], [], []
        for node_count in unique_nodes:
            times = np.array([
                pt for m, n, _, pt, _ in records if m == mode and n == node_count
            ])
            mean_t.append(times.mean())
            std_t.append(times.std())
            counts = np.array([
                ac for m, n, _, _, ac in records
                if m == mode and n == node_count and ac != NO_PLAN
            ], dtype=float)
            mean_ac.append(counts.mean() if counts.size else float('nan'))
            std_ac.append(counts.std() if counts.size else 0.0)

        mean_t, std_t = np.array(mean_t), np.array(std_t)
        mean_ac, std_ac = np.array(mean_ac), np.array(std_ac)

        ax1.plot(unique_nodes, mean_t, marker='o', linestyle='-', markersize=3,
                 label=MODE_LABELS[mode])
        ax1.fill_between(unique_nodes, mean_t - std_t, mean_t + std_t, alpha=0.2)
        ax2.plot(unique_nodes, mean_ac, marker='o', linestyle='-', markersize=3,
                 label=MODE_LABELS[mode])
        ax2.fill_between(unique_nodes, mean_ac - std_ac, mean_ac + std_ac, alpha=0.2)

    ax1.set_xlabel('Effective Map Size (nodes)')
    ax1.set_ylabel('Mean Planning Time (seconds)')
    ax1.set_title('Combined Stress Test: Planning Time vs. Map Size')
    ax1.legend()
    ax1.grid(True)

    ax2.set_xlabel('Effective Map Size (nodes)')
    ax2.set_ylabel('Mean Plan Length (actions)')
    ax2.set_ylim(bottom=0)
    ax2.set_title('Combined Stress Test: Plan Length vs. Map Size')
    ax2.legend()
    ax2.grid(True)

    plot_path = folder / 'planning_time_combined.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def runner(n_runs: int, mode: str, min_nodes: int = MIN_NODES, max_nodes: int = MAX_NODES, search: str = 'astar(blind())', out_dir: Path | None = None) -> Path:
    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder = out_dir if out_dir is not None else (REPO_ROOT / 'results' / 'scalability_combined' / date)
    folder.mkdir(parents=True, exist_ok=True)

    modes = ['adaptive', 'no-adaptation'] if mode == 'both' else [mode]
    records = []
    node_sizes = range(min_nodes, max_nodes + NODES_STEP, NODES_STEP)

    for current_mode in modes:
        print(f'\n--- {MODE_LABELS[current_mode]} ---')
        for n_nodes in node_sizes:
            for run_id in range(n_runs):
                records.append(run_single(folder, current_mode, run_id, n_nodes, search))

    folder_mode = folder / mode
    csv_path = folder / 'planning_times.csv'
    save_results(folder_mode, records)
    print(f'\nResults saved to {csv_path}')
    plot_results(folder_mode, records, modes)
    return csv_path


def main():
    parser = argparse.ArgumentParser(
        description='Experiment D: Combined stress test — maximum complexity, vary map size.')
    parser.add_argument(
        '--mode',
        choices=['adaptive', 'no-adaptation', 'both'],
        default='adaptive',
        help=(
            'Experiment variant to run. '
            '"adaptive" reproduces the current PLANTA flow, '
            '"no-adaptation" uses the combined no-adaptation domain directly, '
            'and "both" runs both variants for comparison.'
        ),
    )
    parser.add_argument(
        '--runs', type=int, default=10, metavar='N',
        help='Number of runs per map size (default: 10)')
    parser.add_argument(
        '--min-nodes', type=int, default=MIN_NODES, metavar='N',
        help=f'Minimum map size in nodes (default: {MIN_NODES})')
    parser.add_argument(
        '--max-nodes', type=int, default=MAX_NODES, metavar='N',
        help=f'Maximum map size in nodes (default: {MAX_NODES})')
    parser.add_argument(
        '--search', default='astar(blind())',
        help='Fast Downward search configuration string.')
    args = parser.parse_args()

    min_nodes = args.min_nodes
    max_nodes = args.max_nodes
    node_count = len(range(min_nodes, max_nodes + NODES_STEP, NODES_STEP))
    print(
        f'Combined stress test: mode={args.mode}, {node_count} map sizes '
        f'({min_nodes}–{max_nodes}, step {NODES_STEP}) × {args.runs} runs'
    )
    runner(args.runs, args.mode, min_nodes=min_nodes, max_nodes=max_nodes, search=args.search)


if __name__ == '__main__':
    main()
