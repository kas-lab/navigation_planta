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

"""Experiment C — Mission action type (MA) scaling.

Varies the number of distinct task action schemas M = {1, 2, 3, 4, 5} on a
fixed 100-node grid map. Measures Fast Downward planning time (reported in
paper) and OWLToPDDL wall-clock time (saved to CSV, not reported).

M=1: baseline (a_move only, navigation.owl + domain_sas.pddl)
M=2: + a_inspect / f_camera_inspection
M=3: + a_deliver / f_manipulation
M=4: + a_recharge / f_recharge_management
M=5: + a_report / f_wireless_comm

Usage:
    python scripts/run_mission_action_scale_scenario.py [--runs N_RUNS]
"""

import argparse
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
    1: REPO_ROOT / 'owl' / 'navigation.owl',
    2: REPO_ROOT / 'owl' / 'experiment_ma_scalability' / 'navigation_ma_2.owl',
    3: REPO_ROOT / 'owl' / 'experiment_ma_scalability' / 'navigation_ma_3.owl',
    4: REPO_ROOT / 'owl' / 'experiment_ma_scalability' / 'navigation_ma_4.owl',
    5: REPO_ROOT / 'owl' / 'experiment_ma_scalability' / 'navigation_ma_5.owl',
}

DOMAIN_FILES = {
    1: REPO_ROOT / 'pddl' / 'domain_sas.pddl',
    2: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_sas_ma_2.pddl',
    3: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_sas_ma_3.pddl',
    4: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_sas_ma_4.pddl',
    5: REPO_ROOT / 'pddl' / 'experiment_ma_scalability' / 'domain_sas_ma_5.pddl',
}

M_VALUES = [1, 2, 3, 4, 5]

# Fixed 100-node map parameters (same as baseline grid experiment)
N_NODES = 100
NODES_SKIP = 0.1
UNCONNECTED_AMOUNT = 0.15
UNSAFE_AMOUNT = 0.25
DARK_AMOUNT = 0.25


class MissionMapGenerator(MapGenerator):
    """MapGenerator extended with secondary mission action types.

    For M action types, generates M-1 designated waypoints (for actions
    inspect, deliver, recharge, report in order) and adds task-completion
    predicates as PDDL goals. The robot must visit each designated waypoint
    and perform the corresponding action as part of the mission.

    Secondary functions carry no QA estimations, so the lessThan comparison
    table size is unchanged across all M values.
    """

    # Ordered sequence of secondary action names (M-1 active for M action types).
    # These must match the predicate names in the PDDL domain files:
    # inspection_waypoint / inspection_done, delivery_waypoint / delivery_done, etc.
    ACTION_NAMES = ['inspection', 'delivery', 'recharge', 'report']
    # PDDL constants that must exist in the merged domain (from OWL ABox)
    FUNCTION_CONSTANTS = {
        'inspection': 'f_camera_inspection',
        'delivery': 'f_manipulation',
        'recharge': 'f_recharge_management',
        'report': 'f_wireless_comm',
    }

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount, m):
        super().__init__(
            num_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
        assert 1 <= m <= 5, f'M must be in 1..5, got {m}'
        self.m = m
        self.active_actions = self.ACTION_NAMES[:m - 1]

    def generate_domain(self):
        super().generate_domain()
        # Create Fluent objects for each secondary action type.
        # These are stored as instance attributes for use in generate_problem.
        for action_name in self.active_actions:
            setattr(self, f'{action_name}_waypoint_fluent',
                    unified_planning.model.Fluent(
                        f'{action_name}_waypoint', BoolType(),
                        wp=self.waypoint_type))
            setattr(self, f'{action_name}_done_fluent',
                    unified_planning.model.Fluent(
                        f'{action_name}_done', BoolType()))

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)

        if not self.active_actions:
            return

        sorted_nodes = sorted(self.graph.nodes)
        n = len(sorted_nodes)
        waypoints = {
            node_id: Object('wp%s' % node_id, self.waypoint_type)
            for node_id in self.graph.nodes
        }

        for i, action_name in enumerate(self.active_actions):
            wp_fluent = getattr(self, f'{action_name}_waypoint_fluent')
            done_fluent = getattr(self, f'{action_name}_done_fluent')

            # Register done predicate (starts false; appears in goal, not init).
            # add_fluent is required for zero-arity predicates used as goals.
            self.problem.add_fluent(done_fluent, default_initial_value=False)

            # Pick designated waypoint at evenly spaced position between
            # start (index 0) and goal (index n-1).
            target_node = sorted_nodes[(i + 1) * n // self.m]
            self.problem.set_initial_value(
                wp_fluent(waypoints[target_node]), True)

            # Add task-completion as goal
            self.problem.add_goal(done_fluent())


def make_generator(m: int) -> MapGenerator:
    """Return the appropriate map generator for mission action count M."""
    if m == 1:
        return MapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT)
    return MissionMapGenerator(
        N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT, m=m)


def run_single(folder: Path, run_id: int, m: int):
    """Run one experiment trial: generate map, run OWLToPDDL, run Fast Downward.

    Returns (m, owltopddl_time, planning_time).
    OWLToPDDL time is saved to CSV but not reported in the paper.
    """
    owl_file = OWL_FILES[m]
    domain_file = DOMAIN_FILES[m]
    run_folder = folder / f'ma{m}_run{run_id}'
    run_folder.mkdir(parents=True, exist_ok=True)

    mg = make_generator(m)
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
        print(f'OWLToPDDL failed (m={m}, run={run_id}): {e.stderr}')
        # Retry once with a fresh map
        return run_single(folder, run_id, m)
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
        f'm={m}  run={run_id:2d}  '
        f'owltopddl={owltopddl_time:.3f}s  '
        f'planning={planning_time:.6f}s'
    )
    return m, owltopddl_time, planning_time


def runner(n_runs: int):
    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder = REPO_ROOT / 'results' / 'scalability_ma' / date
    folder.mkdir(parents=True, exist_ok=True)

    records = []  # list of (m, owltopddl_time, planning_time)

    for m in M_VALUES:
        print(f'\n--- M = {m} ---')
        for run_id in range(n_runs):
            result = run_single(folder, run_id, m)
            records.append(result)

    # Save CSV
    arr = np.array(
        records,
        dtype=[('m', 'i4'), ('owltopddl_time', 'f8'), ('planning_time', 'f8')],
    )
    csv_path = folder / 'planning_times.csv'
    np.savetxt(
        csv_path, arr,
        delimiter=',',
        header='m,owltopddl_time,planning_time',
        comments='',
    )
    print(f'\nResults saved to {csv_path}')

    # Plot: planning time vs M (mean ± std)
    unique_m = np.array(M_VALUES)
    mean_t, std_t = [], []
    for m in unique_m:
        times = arr['planning_time'][arr['m'] == m]
        mean_t.append(times.mean())
        std_t.append(times.std())
    mean_t = np.array(mean_t)
    std_t = np.array(std_t)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(unique_m, mean_t, marker='o', linestyle='-', color='b', label='Avg Planning Time')
    ax.errorbar(unique_m, mean_t, yerr=std_t, fmt='o', color='r', capsize=5, label='Std Dev')
    ax.set_xlabel('Number of Mission Action Types (M)')
    ax.set_ylabel('Mean Planning Time (seconds)')
    ax.set_title('Planning Time vs. Number of Mission Action Types')
    ax.set_xticks(M_VALUES)
    ax.legend()
    ax.grid(True)
    plot_path = folder / 'planning_time_ma_scale.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def main():
    parser = argparse.ArgumentParser(
        description='Experiment C: MA scaling — vary number of mission action types M.')
    parser.add_argument(
        '--runs', type=int, default=10, metavar='N',
        help='Number of runs per M value (default: 10)')
    args = parser.parse_args()

    print(f'MA scaling experiment: M={M_VALUES}, runs={args.runs}, nodes={N_NODES}')
    runner(args.runs)


if __name__ == '__main__':
    main()
