#!/usr/bin/env python3

"""Experiment B — Corridor type scaling with optional no-adaptation mode."""

import argparse
import random
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import unified_planning
from unified_planning.shortcuts import BoolType, Object

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

from navigation_planta.experiment_runner import (
    SweepExperimentConfig,
    execute_pddl_case,
    run_sweep_experiment,
)
from navigation_planta.map_generator import MapGenerator
from navigation_planta.no_adaptation import NoAdaptationMapGenerator
from navigation_planta.utils import (
    NO_PLAN,
    plot_memory_boxplot,
)

OWL_FILES = {
    2: REPO_ROOT / 'owl' / 'navigation.owl',
    3: REPO_ROOT / 'owl' / 'experiment_qa_scalability' / 'navigation_ct_3.owl',
    4: REPO_ROOT / 'owl' / 'experiment_qa_scalability' / 'navigation_ct_4.owl',
}

ADAPTIVE_DOMAIN_FILES = {
    2: REPO_ROOT / 'pddl' / 'domain_sas.pddl',
    3: REPO_ROOT / 'pddl' / 'experiment_qa_scalability' / 'domain_sas_ct_3.pddl',
    4: REPO_ROOT / 'pddl' / 'experiment_qa_scalability' / 'domain_sas_ct_4.pddl',
}

BASELINE_DOMAIN_FILE = REPO_ROOT / 'pddl' / 'domain_no_sas.pddl'

MODE_LABELS = {
    'adaptive': 'Adaptive',
    'no-adaptation': 'No adaptation',
}

K_VALUES = [2, 3, 4]

N_NODES = 100
NODES_SKIP = 0.1
UNCONNECTED_AMOUNT = 0.15
UNSAFE_AMOUNT = 0.25
DARK_AMOUNT = 0.25
OUTDOOR_AMOUNT = 0.20
DUSTY_AMOUNT = 0.20


class ExtendedMapGenerator(MapGenerator):
    """MapGenerator extended with outdoor and dusty corridor types."""

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount,
                 outdoor_amount=0.0, dusty_amount=0.0):
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
        if self.outdoor_amount > 0:
            self.outdoor_requirement = unified_planning.model.Fluent(
                'outdoor_requirement', BoolType(),
                wp1=self.waypoint_type,
                wp2=self.waypoint_type,
                v=self.numerical_object_type)
        if self.dusty_amount > 0:
            self.dust_requirement = unified_planning.model.Fluent(
                'dust_requirement', BoolType(),
                wp1=self.waypoint_type,
                wp2=self.waypoint_type,
                v=self.numerical_object_type)

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)

        waypoints = {
            node_id: Object(f'wp{node_id}', self.waypoint_type)
            for node_id in self.graph.nodes
        }
        zero_decimal = Object('0.0_decimal', self.numerical_object_type)
        zero_eight_decimal = Object('0.8_decimal', self.numerical_object_type)

        for u, v in self.graph.edges:
            if self.outdoor_amount > 0:
                req = zero_eight_decimal if self.graph[u][v].get('outdoor') else zero_decimal
                self.problem.set_initial_value(
                    self.outdoor_requirement(waypoints[u], waypoints[v], req), True)
                self.problem.set_initial_value(
                    self.outdoor_requirement(waypoints[v], waypoints[u], req), True)
            if self.dusty_amount > 0:
                req = zero_eight_decimal if self.graph[u][v].get('dusty') else zero_decimal
                self.problem.set_initial_value(
                    self.dust_requirement(waypoints[u], waypoints[v], req), True)
                self.problem.set_initial_value(
                    self.dust_requirement(waypoints[v], waypoints[u], req), True)


def make_generator(mode: str, k: int) -> MapGenerator:
    if mode == 'no-adaptation':
        return NoAdaptationMapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT)
    if k == 2:
        return MapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT)
    if k == 3:
        return ExtendedMapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT,
            outdoor_amount=OUTDOOR_AMOUNT)
    if k == 4:
        return ExtendedMapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT,
            outdoor_amount=OUTDOOR_AMOUNT, dusty_amount=DUSTY_AMOUNT)
    raise ValueError(f'Unsupported K={k}')


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


def run_single(folder: Path, mode: str, run_id: int, k: int, search: str = 'lazy_greedy([ff()], preferred=[ff()])'):
    """Run one experiment trial."""
    def prepare_problem(run_folder: Path, current_mode: str) -> Path:
        mg = make_generator(current_mode, k)
        mg.generate_connected_grid_map()

        problem_file = run_folder / (
            'problem.pddl' if current_mode == 'adaptive' else 'problem_no_sas.pddl')
        mg.generate_domain_problem_files(save_problem=True, problem_filename=problem_file)
        mg.plot_graph(show_plot=False, save_file=True, filename=run_folder / 'map.png')
        return problem_file

    record = execute_pddl_case(
        folder=folder,
        mode=mode,
        case_id=f'ct{k}_run{run_id}',
        x_value=k,
        search=search,
        prepare_problem=prepare_problem,
        adaptive_preprocessor=lambda problem_file, run_folder: run_adaptive_preprocessor(
            OWL_FILES[k],
            ADAPTIVE_DOMAIN_FILES[k],
            problem_file,
            run_folder,
        ),
        baseline_domain=BASELINE_DOMAIN_FILE,
        include_owltopddl_time=True,
    )

    print(
        f'mode={mode:<13} k={k} run={run_id:2d} '
        f'owltopddl={record.owltopddl_time:.3f}s planning={record.planning_time:.6f}s '
        f'actions={record.action_count} peak_memory={record.peak_memory:.2f}MB'
    )
    return record

def plot_results(folder: Path, records, modes):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    for mode in modes:
        mean_t, std_t, mean_ac, std_ac = [], [], [], []
        for k in K_VALUES:
            times = np.array([
                record.planning_time
                for record in records
                if record.mode == mode and record.x_value == k
            ])
            mean_t.append(times.mean())
            std_t.append(times.std())
            counts = np.array([
                record.action_count
                for record in records
                if record.mode == mode and record.x_value == k and record.action_count != NO_PLAN
            ], dtype=float)
            mean_ac.append(counts.mean() if counts.size else float('nan'))
            std_ac.append(counts.std() if counts.size else 0.0)

        mean_t, std_t = np.array(mean_t), np.array(std_t)
        mean_ac, std_ac = np.array(mean_ac), np.array(std_ac)

        ax1.plot(K_VALUES, mean_t, marker='o', linestyle='-', label=MODE_LABELS[mode])
        ax1.fill_between(K_VALUES, mean_t - std_t, mean_t + std_t, alpha=0.2)
        ax2.plot(K_VALUES, mean_ac, marker='o', linestyle='-', label=MODE_LABELS[mode])
        ax2.fill_between(K_VALUES, mean_ac - std_ac, mean_ac + std_ac, alpha=0.2)

    ax1.set_xlabel('Number of Corridor Types (K)')
    ax1.set_ylabel('Mean Planning Time (seconds)')
    ax1.set_title('Planning Time vs. Number of Corridor Types')
    ax1.set_xticks(K_VALUES)
    ax1.legend()
    ax1.grid(True)

    ax2.set_xlabel('Number of Corridor Types (K)')
    ax2.set_ylabel('Mean Plan Length (actions)')
    ax2.set_ylim(bottom=0)
    ax2.set_title('Plan Length vs. Number of Corridor Types')
    ax2.set_xticks(K_VALUES)
    ax2.legend()
    ax2.grid(True)

    plot_path = folder / 'planning_time_ct_scale.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def runner(n_runs: int, mode: str, search: str = 'lazy_greedy([ff()], preferred=[ff()])', out_dir: Path | None = None) -> Path:
    config = SweepExperimentConfig(
        results_root=REPO_ROOT / 'results' / 'scalability_ct',
        mode_labels=MODE_LABELS,
        x_values=K_VALUES,
        x_name='k',
        time_name='planning_time',
    )
    return run_sweep_experiment(
        config,
        n_runs=n_runs,
        mode=mode,
        run_one=run_single,
        plot_results=plot_results,
        after_run=lambda folder_mode, records, _modes: plot_memory_boxplot(
            folder_mode, records, MODE_LABELS, filename='peak_memory_boxplot.png'),
        value_printer=lambda k: f'  K = {k}',
        search=search,
        out_dir=out_dir,
    )


def main():
    parser = argparse.ArgumentParser(
        description='Experiment B: CT scaling — vary number of corridor types K.')
    parser.add_argument(
        '--mode',
        choices=['adaptive', 'no-adaptation', 'both'],
        default='adaptive',
        help=(
            'Experiment variant to run. '
            '"adaptive" reproduces the current PLANTA flow, '
            '"no-adaptation" uses pddl/domain_no_sas.pddl directly, '
            'and "both" runs both variants for comparison.'
        ),
    )
    parser.add_argument(
        '--runs', type=int, default=10, metavar='N',
        help='Number of runs per K value (default: 10)')
    parser.add_argument(
        '--search', default='astar(blind())',
        help='Fast Downward search configuration string (default: astar blind)')
    args = parser.parse_args()

    print(f'CT scaling experiment: mode={args.mode}, K={K_VALUES}, runs={args.runs}, nodes={N_NODES}')
    runner(args.runs, args.mode, args.search)


if __name__ == '__main__':
    main()
