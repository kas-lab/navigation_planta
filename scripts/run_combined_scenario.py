#!/usr/bin/env python3

# Copyright 2025 Gustavo Rezende Silva
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Experiment D — Combined stress test.

Fixes domain at maximum complexity (N=15 FDs, K=4 corridor types, M=5 mission
action types) and varies map size from 10 to 1000 nodes in steps of 10 — the
same range as the baseline grid experiment.  Measures Fast Downward planning
time (reported in paper) and OWLToPDDL wall-clock time (saved to CSV, not
reported).

Usage:
    python scripts/run_combined_scenario.py [--runs N_RUNS]
"""

import argparse
import random
import sys
import time
import subprocess
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import unified_planning
from unified_planning.shortcuts import Object, BoolType

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

sys.path.insert(0, str(SCRIPT_DIR))
from map_generator import MapGenerator  # noqa: E402

OWL_FILE = REPO_ROOT / 'owl' / 'experiment_combined_scalability' / 'navigation_combined.owl'
DOMAIN_FILE = REPO_ROOT / 'pddl' / 'experiment_combined_scalability' / 'domain_sas_combined.pddl'

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
    """MapGenerator at maximum complexity: K=4 corridor types + M=5 mission actions.

    Outdoor and dusty corridor fractions are assigned independently using the
    same random-sampling mechanism as ExtendedMapGenerator (Experiment B).
    Four secondary action types (inspection, delivery, recharge, report) each
    get a designated waypoint evenly spaced between start and goal, following
    MissionMapGenerator (Experiment C).

    This is a direct subclass of MapGenerator (not a multi-inheritance chain)
    to avoid MRO complexity from inheriting two classes that both override
    generate_domain and generate_problem.
    """

    # Predicate name stems matching the PDDL domain predicates exactly:
    # inspection_waypoint / inspection_done, delivery_waypoint / delivery_done, etc.
    ACTION_NAMES = ['inspection', 'delivery', 'recharge', 'report']
    M = 5  # total action types including a_move

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount,
                 outdoor_amount=OUTDOOR_AMOUNT, dusty_amount=DUSTY_AMOUNT):
        super().__init__(
            num_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
        self.outdoor_amount = outdoor_amount
        self.dusty_amount = dusty_amount

    def assign_dark_unsafe_corridors(self, graph):
        # Initialise new attributes before calling parent so all edges have them
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
        # CT=4: outdoor and dusty corridor requirement fluents
        self.outdoor_requirement = unified_planning.model.Fluent(
            'outdoor_requirement', BoolType(),
            wp1=self.waypoint_type, wp2=self.waypoint_type,
            v=self.numerical_object_type)
        self.dust_requirement = unified_planning.model.Fluent(
            'dust_requirement', BoolType(),
            wp1=self.waypoint_type, wp2=self.waypoint_type,
            v=self.numerical_object_type)
        # MA=5: task waypoint and completion fluents for each secondary action
        for action_name in self.ACTION_NAMES:
            setattr(self, f'{action_name}_waypoint_fluent',
                    unified_planning.model.Fluent(
                        f'{action_name}_waypoint', BoolType(),
                        wp=self.waypoint_type))
            setattr(self, f'{action_name}_done_fluent',
                    unified_planning.model.Fluent(
                        f'{action_name}_done', BoolType()))

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)

        waypoints = {
            node_id: Object('wp%s' % node_id, self.waypoint_type)
            for node_id in self.graph.nodes
        }
        zero_decimal = Object('0.0_decimal', self.numerical_object_type)
        zero_eight_decimal = Object('0.8_decimal', self.numerical_object_type)

        # CT=4: set outdoor and dusty corridor requirements for every edge
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

        # MA=5: assign designated waypoints for each secondary action and add goals
        sorted_nodes = sorted(self.graph.nodes)
        n = len(sorted_nodes)

        for i, action_name in enumerate(self.ACTION_NAMES):
            wp_fluent = getattr(self, f'{action_name}_waypoint_fluent')
            done_fluent = getattr(self, f'{action_name}_done_fluent')

            # Register the zero-arity done predicate so it can be used as a goal
            self.problem.add_fluent(done_fluent, default_initial_value=False)

            # Evenly space 4 waypoints at 20%, 40%, 60%, 80% of sorted node list
            target_node = sorted_nodes[(i + 1) * n // self.M]
            self.problem.set_initial_value(wp_fluent(waypoints[target_node]), True)
            self.problem.add_goal(done_fluent())


def run_single(folder: Path, run_id: int, n_nodes: int):
    """Run one trial for a given map size.

    Returns (n_nodes_resulting, owltopddl_time, planning_time).
    OWLToPDDL time is saved to CSV but not reported in the paper.
    """
    n_nodes_resulting = n_nodes - int(n_nodes * NODES_SKIP)
    run_folder = folder / f'n{n_nodes}_run{run_id}'
    run_folder.mkdir(parents=True, exist_ok=True)

    mg = CombinedMapGenerator(n_nodes, NODES_SKIP, UNCONNECTED_AMOUNT,
                               UNSAFE_AMOUNT, DARK_AMOUNT)
    mg.generate_connected_grid_map()

    problem_file = run_folder / 'problem.pddl'
    mg.generate_domain_problem_files(save_problem=True, problem_filename=problem_file)
    mg.plot_graph(show_plot=False, save_file=True, filename=run_folder / 'map.png')

    domain_out = run_folder / 'domain_created.pddl'
    problem_out = run_folder / 'problem_created.pddl'

    # --- OWL-to-PDDL (timed for CSV, not reported in paper) ---
    owltopddl_start = time.perf_counter()
    try:
        subprocess.run(
            [
                'OWLToPDDL.sh',
                f'--owl={OWL_FILE}',
                '--tBox',
                f'--inDomain={DOMAIN_FILE}',
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
    except subprocess.CalledProcessError as e:
        print(f'OWLToPDDL failed (n={n_nodes}, run={run_id}): {e.stderr}')
        # Retry once with a fresh map
        return run_single(folder, run_id, n_nodes)
    owltopddl_time = time.perf_counter() - owltopddl_start

    # --- Fast Downward (timed, reported in paper) ---
    fd_start = time.perf_counter()
    subprocess.run(
        [
            'fast-downward.py',
            str(domain_out),
            str(problem_out),
            '--search', 'astar(ff())',
        ],
        capture_output=True,
        check=True,
    )
    planning_time = time.perf_counter() - fd_start

    print(
        f'n={n_nodes:4d} (eff={n_nodes_resulting:3d})  run={run_id:2d}  '
        f'owltopddl={owltopddl_time:.3f}s  planning={planning_time:.6f}s'
    )
    return n_nodes_resulting, owltopddl_time, planning_time


def runner(n_runs: int, min_nodes: int = MIN_NODES, max_nodes: int = MAX_NODES):
    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder = REPO_ROOT / 'results' / 'scalability_combined' / date
    folder.mkdir(parents=True, exist_ok=True)

    records = []
    node_sizes = range(min_nodes, max_nodes + NODES_STEP, NODES_STEP)

    for n_nodes in node_sizes:
        for run_id in range(n_runs):
            result = run_single(folder, run_id, n_nodes)
            records.append(result)

    # Save CSV
    arr = np.array(
        records,
        dtype=[('nodes', 'i4'), ('owltopddl_time', 'f8'), ('planning_time', 'f8')],
    )
    csv_path = folder / 'planning_times.csv'
    np.savetxt(
        csv_path, arr,
        delimiter=',',
        header='nodes,owltopddl_time,planning_time',
        comments='',
    )
    print(f'\nResults saved to {csv_path}')

    # Plot: planning time vs effective node count (mean ± std shaded)
    unique_nodes = np.array(sorted(set(arr['nodes'])))
    mean_t = np.array([arr['planning_time'][arr['nodes'] == n].mean() for n in unique_nodes])
    std_t  = np.array([arr['planning_time'][arr['nodes'] == n].std()  for n in unique_nodes])

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(unique_nodes, mean_t, marker='o', linestyle='-', color='b',
            markersize=3, label='Combined (N=15, K=4, M=5)')
    ax.fill_between(unique_nodes, mean_t - std_t, mean_t + std_t,
                    alpha=0.2, color='b')
    ax.set_xlabel('Effective Map Size (nodes)')
    ax.set_ylabel('Mean Planning Time (seconds)')
    ax.set_title('Combined Stress Test: Planning Time vs. Map Size')
    ax.legend()
    ax.grid(True)
    plot_path = folder / 'planning_time_combined.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def main():
    parser = argparse.ArgumentParser(
        description='Experiment D: Combined stress test — maximum complexity, vary map size.')
    parser.add_argument(
        '--runs', type=int, default=10, metavar='N',
        help='Number of runs per map size (default: 10)')
    parser.add_argument(
        '--min-nodes', type=int, default=MIN_NODES, metavar='N',
        help=f'Minimum map size in nodes (default: {MIN_NODES})')
    parser.add_argument(
        '--max-nodes', type=int, default=MAX_NODES, metavar='N',
        help=f'Maximum map size in nodes (default: {MAX_NODES})')
    args = parser.parse_args()

    min_nodes = args.min_nodes
    max_nodes = args.max_nodes
    node_count = len(range(min_nodes, max_nodes + NODES_STEP, NODES_STEP))
    print(
        f'Combined stress test: {node_count} map sizes '
        f'({min_nodes}–{max_nodes}, step {NODES_STEP}) × {args.runs} runs'
    )
    runner(args.runs, min_nodes=min_nodes, max_nodes=max_nodes)


if __name__ == '__main__':
    main()
