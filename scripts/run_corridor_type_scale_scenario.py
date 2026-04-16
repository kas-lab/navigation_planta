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

"""Experiment B — Corridor type (CT) scaling.

Varies the number of distinct corridor QA dimensions K = {2, 3, 4} on a
fixed 100-node grid map. Measures Fast Downward planning time (reported
in paper) and OWLToPDDL wall-clock time (saved to CSV, not reported).

K=2: dark + unsafe (baseline navigation.owl + domain_sas.pddl)
K=3: + outdoor (navigation_ct_3.owl + domain_sas_ct_3.pddl)
K=4: + outdoor + dusty (navigation_ct_4.owl + domain_sas_ct_4.pddl)

Usage:
    python scripts/run_corridor_type_scale_scenario.py [--runs N_RUNS]
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

OWL_FILES = {
    2: REPO_ROOT / 'owl' / 'navigation.owl',
    3: REPO_ROOT / 'owl' / 'experiment_qa_scalability' / 'navigation_ct_3.owl',
    4: REPO_ROOT / 'owl' / 'experiment_qa_scalability' / 'navigation_ct_4.owl',
}

DOMAIN_FILES = {
    2: REPO_ROOT / 'pddl' / 'domain_sas.pddl',
    3: REPO_ROOT / 'pddl' / 'experiment_qa_scalability' / 'domain_sas_ct_3.pddl',
    4: REPO_ROOT / 'pddl' / 'experiment_qa_scalability' / 'domain_sas_ct_4.pddl',
}

K_VALUES = [2, 3, 4]

# Fixed 100-node map parameters (same as baseline grid experiment)
N_NODES = 100
NODES_SKIP = 0.1
UNCONNECTED_AMOUNT = 0.15
UNSAFE_AMOUNT = 0.25
DARK_AMOUNT = 0.25
OUTDOOR_AMOUNT = 0.20   # fraction of edges assigned as outdoor (K >= 3)
DUSTY_AMOUNT = 0.20     # fraction of edges assigned as dusty   (K >= 4)


class ExtendedMapGenerator(MapGenerator):
    """MapGenerator extended with outdoor and dusty corridor types.

    Adds two optional corridor properties beyond the baseline dark/unsafe:
    - outdoor: requires outdoor_robustness QA from the FD
    - dusty: requires dust_robustness QA from the FD

    Each active corridor type is independently assigned to a random
    fraction of edges (same mechanism as dark_amount / unsafe_amount).
    """

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount,
                 outdoor_amount=0.0, dusty_amount=0.0):
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

        # Re-derive waypoints and numeric objects (same naming scheme as parent)
        waypoints = {
            node_id: Object('wp%s' % node_id, self.waypoint_type)
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


def make_generator(k: int) -> MapGenerator:
    """Return the appropriate map generator for corridor type count K."""
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


def run_single(folder: Path, run_id: int, k: int):
    """Run one experiment trial: generate map, run OWLToPDDL, run Fast Downward.

    Returns (k, owltopddl_time, planning_time).
    OWLToPDDL time is saved to CSV but not reported in the paper.
    """
    owl_file = OWL_FILES[k]
    domain_file = DOMAIN_FILES[k]
    run_folder = folder / f'ct{k}_run{run_id}'
    run_folder.mkdir(parents=True, exist_ok=True)

    mg = make_generator(k)
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
    except subprocess.CalledProcessError as e:
        print(f'OWLToPDDL failed (k={k}, run={run_id}): {e.stderr}')
        # Retry once with a fresh map
        return run_single(folder, run_id, k)
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
        f'k={k}  run={run_id:2d}  '
        f'owltopddl={owltopddl_time:.3f}s  '
        f'planning={planning_time:.6f}s'
    )
    return k, owltopddl_time, planning_time


def runner(n_runs: int):
    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder = REPO_ROOT / 'results' / 'scalability_ct' / date
    folder.mkdir(parents=True, exist_ok=True)

    records = []  # list of (k, owltopddl_time, planning_time)

    for k in K_VALUES:
        print(f'\n--- K = {k} ---')
        for run_id in range(n_runs):
            result = run_single(folder, run_id, k)
            records.append(result)

    # Save CSV
    arr = np.array(
        records,
        dtype=[('k', 'i4'), ('owltopddl_time', 'f8'), ('planning_time', 'f8')],
    )
    csv_path = folder / 'planning_times.csv'
    np.savetxt(
        csv_path, arr,
        delimiter=',',
        header='k,owltopddl_time,planning_time',
        comments='',
    )
    print(f'\nResults saved to {csv_path}')

    # Plot: planning time vs K (mean ± std)
    unique_k = np.array(K_VALUES)
    mean_t, std_t = [], []
    for k in unique_k:
        times = arr['planning_time'][arr['k'] == k]
        mean_t.append(times.mean())
        std_t.append(times.std())
    mean_t = np.array(mean_t)
    std_t = np.array(std_t)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(unique_k, mean_t, marker='o', linestyle='-', color='b', label='Avg Planning Time')
    ax.errorbar(unique_k, mean_t, yerr=std_t, fmt='o', color='r', capsize=5, label='Std Dev')
    ax.set_xlabel('Number of Corridor Types (K)')
    ax.set_ylabel('Mean Planning Time (seconds)')
    ax.set_title('Planning Time vs. Number of Corridor Types')
    ax.set_xticks([2, 3, 4])
    ax.legend()
    ax.grid(True)
    plot_path = folder / 'planning_time_ct_scale.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def main():
    parser = argparse.ArgumentParser(
        description='Experiment B: CT scaling — vary number of corridor types K.')
    parser.add_argument(
        '--runs', type=int, default=10, metavar='N',
        help='Number of runs per K value (default: 10)')
    args = parser.parse_args()

    print(f'CT scaling experiment: K={K_VALUES}, runs={args.runs}, nodes={N_NODES}')
    runner(args.runs)


if __name__ == '__main__':
    main()
