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

"""Experiment A — FunctionDesign (FD) scaling.

Varies the number of FunctionDesigns N = {2, 4, 6, 8, 10, 12, 15} on a fixed
100-node grid map. Measures Fast Downward planning time (reported in paper) and
OWLToPDDL wall-clock time (saved to CSV, not reported).

Usage:
    python scripts/run_fd_scale_scenario.py [--runs N_RUNS]
"""

import argparse
import sys
import time
import subprocess
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

sys.path.insert(0, str(SCRIPT_DIR))
from map_generator import MapGenerator  # noqa: E402

OWL_FILES = {
    2:  REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_2.owl',
    4:  REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_4.owl',
    6:  REPO_ROOT / 'owl' / 'navigation.owl',
    8:  REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_8.owl',
    10: REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_10.owl',
    12: REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_12.owl',
    15: REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_15.owl',
}

DOMAIN_FILE = REPO_ROOT / 'pddl' / 'domain_sas.pddl'

N_FD_VALUES = [2, 4, 6, 8, 10, 12, 15]

# Fixed 100-node map parameters (same as baseline grid experiment)
N_NODES = 100
NODES_SKIP = 0.1
UNCONNECTED_AMOUNT = 0.15
UNSAFE_AMOUNT = 0.25
DARK_AMOUNT = 0.25


def run_single(folder: Path, run_id: int, n_fd: int):
    """Run one experiment trial: generate map, run OWLToPDDL, run Fast Downward.

    Returns (n_fd, owltopddl_time, planning_time).
    OWLToPDDL time is saved to CSV but not reported in the paper.
    """
    owl_file = OWL_FILES[n_fd]
    run_folder = folder / f'fd{n_fd}_run{run_id}'
    run_folder.mkdir(parents=True, exist_ok=True)

    # Generate a fresh 100-node map for this run (preserves stochastic variation)
    mg = MapGenerator(N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT)
    mg.generate_connected_grid_map()

    problem_file = run_folder / 'problem.pddl'
    mg.generate_domain_problem_files(
        save_problem=True,
        problem_filename=problem_file,
    )
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
        print(f'OWLToPDDL failed (n_fd={n_fd}, run={run_id}): {e.stderr}')
        # Retry once with a fresh map
        return run_single(folder, run_id, n_fd)
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
        f'n_fd={n_fd:2d}  run={run_id:2d}  '
        f'owltopddl={owltopddl_time:.3f}s  '
        f'planning={planning_time:.6f}s'
    )
    return n_fd, owltopddl_time, planning_time


def runner(n_runs: int):
    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder = REPO_ROOT / 'results' / 'scalability_fd' / date
    folder.mkdir(parents=True, exist_ok=True)

    records = []  # list of (n_fd, owltopddl_time, planning_time)

    for n_fd in N_FD_VALUES:
        print(f'\n--- N_FD = {n_fd} ---')
        for run_id in range(n_runs):
            result = run_single(folder, run_id, n_fd)
            records.append(result)

    # Save CSV
    arr = np.array(
        records,
        dtype=[('n_fd', 'i4'), ('owltopddl_time', 'f8'), ('planning_time', 'f8')],
    )
    csv_path = folder / 'planning_times.csv'
    np.savetxt(
        csv_path, arr,
        delimiter=',',
        header='n_fd,owltopddl_time,planning_time',
        comments='',
    )
    print(f'\nResults saved to {csv_path}')

    # Plot: planning time vs n_fd (mean ± std)
    unique_n = np.array(N_FD_VALUES)
    mean_t, std_t = [], []
    for n in unique_n:
        times = arr['planning_time'][arr['n_fd'] == n]
        mean_t.append(times.mean())
        std_t.append(times.std())
    mean_t = np.array(mean_t)
    std_t = np.array(std_t)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(unique_n, mean_t, marker='o', linestyle='-', color='b', label='Avg Planning Time')
    ax.errorbar(unique_n, mean_t, yerr=std_t, fmt='o', color='r', capsize=5, label='Std Dev')
    ax.set_xlabel('Number of FunctionDesigns (N)')
    ax.set_ylabel('Mean Planning Time (seconds)')
    ax.set_title('Planning Time vs. Number of FunctionDesigns')
    ax.legend()
    ax.grid(True)
    plot_path = folder / 'planning_time_fd_scale.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def main():
    parser = argparse.ArgumentParser(
        description='Experiment A: FD scaling — vary N FunctionDesigns on a fixed 100-node map.')
    parser.add_argument(
        '--runs', type=int, default=10, metavar='N',
        help='Number of runs per N_FD value (default: 10)')
    args = parser.parse_args()

    print(f'FD scaling experiment: N_FD={N_FD_VALUES}, runs={args.runs}, nodes={N_NODES}')
    runner(args.runs)


if __name__ == '__main__':
    main()
