#!/usr/bin/env python3

"""Experiment A — FunctionDesign scaling with optional no-adaptation mode."""

import argparse
import subprocess
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

from navigation_planta.map_generator import MapGenerator
from navigation_planta.no_adaptation import NoAdaptationMapGenerator
from navigation_planta.utils import count_plan_actions, NO_PLAN

OWL_FILES = {
    2: REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_2.owl',
    4: REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_4.owl',
    6: REPO_ROOT / 'owl' / 'navigation.owl',
    8: REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_8.owl',
    10: REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_10.owl',
    12: REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_12.owl',
    15: REPO_ROOT / 'owl' / 'experiment_fd_scalability' / 'navigation_fd_15.owl',
}

ADAPTIVE_DOMAIN_FILE = REPO_ROOT / 'pddl' / 'domain_sas.pddl'
BASELINE_DOMAIN_FILE = REPO_ROOT / 'pddl' / 'domain_no_sas.pddl'

MODE_LABELS = {
    'adaptive': 'Adaptive',
    'no-adaptation': 'No adaptation',
}

N_FD_VALUES = [2, 4, 6, 8, 10, 12, 15]

N_NODES = 100
NODES_SKIP = 0.1
UNCONNECTED_AMOUNT = 0.15
UNSAFE_AMOUNT = 0.25
DARK_AMOUNT = 0.25


def make_generator(mode):
    generator_class = MapGenerator if mode == 'adaptive' else NoAdaptationMapGenerator
    return generator_class(
        N_NODES,
        NODES_SKIP,
        UNCONNECTED_AMOUNT,
        UNSAFE_AMOUNT,
        DARK_AMOUNT,
    )


def run_adaptive_preprocessor(owl_file, problem_file, run_folder):
    domain_out = run_folder / 'domain_created.pddl'
    problem_out = run_folder / 'problem_created.pddl'
    subprocess.run(
        [
            'OWLToPDDL.sh',
            f'--owl={owl_file}',
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


def run_single(folder: Path, mode: str, run_id: int, n_fd: int, search: str = 'lazy_greedy([ff()], preferred=[ff()])'):
    """Run one experiment trial."""
    run_folder = folder / mode / f'fd{n_fd}_run{run_id}'
    run_folder.mkdir(parents=True, exist_ok=True)

    mg = make_generator(mode)
    mg.generate_connected_grid_map()

    problem_file = run_folder / (
        'problem.pddl' if mode == 'adaptive' else 'problem_no_sas.pddl')
    mg.generate_domain_problem_files(save_problem=True, problem_filename=problem_file)
    mg.plot_graph(show_plot=False, save_file=True, filename=run_folder / 'map.png')

    if mode == 'adaptive':
        owl_file = OWL_FILES[n_fd]
        owltopddl_start = time.perf_counter()
        try:
            domain_for_planner, problem_for_planner = run_adaptive_preprocessor(
                owl_file,
                problem_file,
                run_folder,
            )
        except subprocess.CalledProcessError as error:
            print(f'OWLToPDDL failed (mode={mode}, n_fd={n_fd}, run={run_id}): {error.stderr}')
            return run_single(folder, mode, run_id, n_fd, search)
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
        f'mode={mode:<13} n_fd={n_fd:2d} run={run_id:2d} '
        f'owltopddl={owltopddl_time:.3f}s planning={planning_time:.6f}s actions={action_count}'
    )
    return mode, n_fd, owltopddl_time, planning_time, action_count


def save_results(folder: Path, records):
    csv_path = folder / 'planning_times.csv'
    single_mode = len({mode for mode, _, _, _, _ in records}) == 1

    if single_mode:
        arr = np.array(
            [(n_fd, owltopddl_time, planning_time, action_count)
             for _, n_fd, owltopddl_time, planning_time, action_count in records],
            dtype=[('n_fd', 'i4'), ('owltopddl_time', 'f8'),
                   ('planning_time', 'f8'), ('action_count', 'i4')],
        )
        np.savetxt(
            csv_path, arr, delimiter=',',
            header='n_fd,owltopddl_time,planning_time,action_count',
            comments='', fmt='%d,%.18e,%.18e,%d',
        )
        return arr

    arr = np.array(
        [(mode, n_fd, owltopddl_time, planning_time, action_count)
         for mode, n_fd, owltopddl_time, planning_time, action_count in records],
        dtype=[('mode', 'U20'), ('n_fd', 'i4'), ('owltopddl_time', 'f8'),
               ('planning_time', 'f8'), ('action_count', 'i4')],
    )
    np.savetxt(
        csv_path, arr, delimiter=',',
        header='mode,n_fd,owltopddl_time,planning_time,action_count',
        comments='', fmt='%s,%d,%.18e,%.18e,%d',
    )
    return arr


def plot_results(folder: Path, records, modes):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    for mode in modes:
        mean_t, std_t, mean_ac, std_ac = [], [], [], []
        for n_fd in N_FD_VALUES:
            times = np.array([
                pt for m, nf, _, pt, _ in records if m == mode and nf == n_fd
            ])
            mean_t.append(times.mean())
            std_t.append(times.std())
            counts = np.array([
                ac for m, nf, _, _, ac in records
                if m == mode and nf == n_fd and ac != NO_PLAN
            ], dtype=float)
            mean_ac.append(counts.mean() if counts.size else float('nan'))
            std_ac.append(counts.std() if counts.size else 0.0)

        mean_t, std_t = np.array(mean_t), np.array(std_t)
        mean_ac, std_ac = np.array(mean_ac), np.array(std_ac)

        ax1.plot(N_FD_VALUES, mean_t, marker='o', linestyle='-', label=MODE_LABELS[mode])
        ax1.fill_between(N_FD_VALUES, mean_t - std_t, mean_t + std_t, alpha=0.2)
        ax2.plot(N_FD_VALUES, mean_ac, marker='o', linestyle='-', label=MODE_LABELS[mode])
        ax2.fill_between(N_FD_VALUES, mean_ac - std_ac, mean_ac + std_ac, alpha=0.2)

    ax1.set_xlabel('Number of FunctionDesigns (N)')
    ax1.set_ylabel('Mean Planning Time (seconds)')
    ax1.set_title('Planning Time vs. Number of FunctionDesigns')
    ax1.legend()
    ax1.grid(True)

    ax2.set_xlabel('Number of FunctionDesigns (N)')
    ax2.set_ylabel('Mean Plan Length (actions)')
    ax2.set_ylim(bottom=0)
    ax2.set_title('Plan Length vs. Number of FunctionDesigns')
    ax2.legend()
    ax2.grid(True)

    plot_path = folder / 'planning_time_fd_scale.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def runner(n_runs: int, mode: str, search: str = 'lazy_greedy([ff()], preferred=[ff()])', out_dir: Path | None = None) -> Path:
    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder = out_dir if out_dir is not None else (REPO_ROOT / 'results' / 'scalability_fd' / date)
    folder.mkdir(parents=True, exist_ok=True)

    modes = ['adaptive', 'no-adaptation'] if mode == 'both' else [mode]
    records = []

    for current_mode in modes:
        print(f'\n--- {MODE_LABELS[current_mode]} ---')
        for n_fd in N_FD_VALUES:
            print(f'  N_FD = {n_fd}')
            for run_id in range(n_runs):
                records.append(run_single(folder, current_mode, run_id, n_fd, search))

    csv_path = folder / 'planning_times.csv'
    save_results(folder, records)
    print(f'\nResults saved to {csv_path}')
    plot_results(folder, records, modes)
    return csv_path


def main():
    parser = argparse.ArgumentParser(
        description='Experiment A: FD scaling — vary N FunctionDesigns on a fixed 100-node map.')
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
        help='Number of runs per N_FD value (default: 10)')
    parser.add_argument(
        '--search', default='lazy_greedy([ff()], preferred=[ff()])',
        help='Fast Downward search configuration string (default: lazy_greedy with ff)')
    args = parser.parse_args()

    print(
        f'FD scaling experiment: mode={args.mode}, N_FD={N_FD_VALUES}, '
        f'runs={args.runs}, nodes={N_NODES}'
    )
    runner(args.runs, args.mode, args.search)


if __name__ == '__main__':
    main()
