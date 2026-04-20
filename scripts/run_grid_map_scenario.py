#!/usr/bin/env python3

"""Grid-map scalability experiment runner.

By default this reproduces the adaptive PLANTA experiment, using OWLToPDDL to
merge the TOMASys ontology with ``pddl/domain_sas.pddl`` before invoking Fast
Downward. It can also run a no-adaptation baseline directly against
``pddl/domain_no_sas.pddl`` so both variants can be compared on the same map
family.
"""

import argparse
import subprocess
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from navigation_planta.map_generator import MapGenerator
from navigation_planta.no_adaptation import NoAdaptationMapGenerator
from navigation_planta.utils import count_plan_actions, NO_PLAN

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

OWL_FILE = REPO_ROOT / 'owl' / 'navigation.owl'
ADAPTIVE_DOMAIN_FILE = REPO_ROOT / 'pddl' / 'domain_sas.pddl'
BASELINE_DOMAIN_FILE = REPO_ROOT / 'pddl' / 'domain_no_sas.pddl'

MODE_LABELS = {
    'adaptive': 'Adaptive',
    'no-adaptation': 'No adaptation',
}


def make_map_generator(
        mode,
        n_nodes,
        nodes_skip,
        unconnected_amount,
        unsafe_amount,
        dark_amount):
    generator_class = MapGenerator if mode == 'adaptive' else NoAdaptationMapGenerator
    return generator_class(
        n_nodes,
        nodes_skip,
        unconnected_amount,
        unsafe_amount,
        dark_amount)


def run_adaptive_preprocessor(problem_input, run_folder):
    domain_output = run_folder / 'domain_created.pddl'
    problem_output = run_folder / 'problem_created.pddl'
    subprocess.run([
        'OWLToPDDL.sh',
        f'--owl={OWL_FILE}',
        '--tBox',
        f'--inDomain={ADAPTIVE_DOMAIN_FILE}',
        f'--outDomain={domain_output}',
        '--aBox',
        f'--inProblem={problem_input}',
        f'--outProblem={problem_output}',
        '--replace-output',
        '--add-num-comparisons',
    ], capture_output=True, text=True, check=True)
    return domain_output, problem_output


def run_single_case(
        folder_name,
        mode,
        run_n,
        n_nodes,
        nodes_skip,
        unconnected_amount,
        unsafe_amount,
        dark_amount,
        search='astar(blind())'):
    n_nodes_resulting = n_nodes - int(n_nodes * nodes_skip)
    map_generator = make_map_generator(
        mode,
        n_nodes,
        nodes_skip,
        unconnected_amount,
        unsafe_amount,
        dark_amount)
    map_generator.generate_connected_grid_map()

    map_folder_name = (
        f'{n_nodes}_{nodes_skip}_{unconnected_amount}_{unsafe_amount}_{dark_amount}_{run_n}'
    )
    run_folder = folder_name / mode / map_folder_name
    run_folder.mkdir(parents=True, exist_ok=True)

    problem_filename = run_folder / (
        'problem.pddl' if mode == 'adaptive' else 'problem_no_sas.pddl')
    map_generator.generate_domain_problem_files(
        save_problem=True,
        problem_filename=problem_filename)

    map_generator.plot_graph(
        show_plot=False,
        save_file=True,
        filename=run_folder / 'map.png')

    if mode == 'adaptive':
        try:
            domain_for_planner, problem_for_planner = run_adaptive_preprocessor(
                problem_filename,
                run_folder)
        except subprocess.CalledProcessError as error:
            print(f'OWLToPDDL failed for nodes={n_nodes} run={run_n}: {error.stderr}')
            return run_single_case(
                folder_name,
                mode,
                run_n,
                n_nodes,
                nodes_skip,
                unconnected_amount,
                unsafe_amount,
                dark_amount,
                search)
    else:
        domain_for_planner = BASELINE_DOMAIN_FILE
        problem_for_planner = problem_filename

    plan_file = run_folder / 'plan'
    start_time = time.perf_counter()
    subprocess.run([
        'fast-downward.py',
        '--plan-file', str(plan_file),
        str(domain_for_planner),
        str(problem_for_planner),
        '--search', search,
    ], check=True)
    elapsed_time = time.perf_counter() - start_time
    action_count = count_plan_actions(plan_file)

    print(
        f'mode={mode:<13} nodes={n_nodes_resulting:<4} '
        f'run={run_n:<2} time={elapsed_time:.6f}s actions={action_count}')
    return mode, n_nodes_resulting, elapsed_time, action_count


def save_results(folder_name, planning_time_list):
    single_mode = len({mode for mode, _, _, _ in planning_time_list}) == 1
    planning_time_csv = folder_name / 'planning_times.csv'

    if single_mode:
        planning_time_array = np.array(
            [(nodes, elapsed_time, action_count)
             for _, nodes, elapsed_time, action_count in planning_time_list],
            dtype=[('nodes', 'i4'), ('time', 'f8'), ('action_count', 'i4')])
        np.savetxt(
            planning_time_csv,
            planning_time_array,
            delimiter=',',
            header='nodes,time,action_count',
            comments='',
            fmt='%d,%.18e,%d')
        return

    planning_time_array = np.array(
        [(mode, nodes, elapsed_time, action_count)
         for mode, nodes, elapsed_time, action_count in planning_time_list],
        dtype=[('mode', 'U20'), ('nodes', 'i4'), ('time', 'f8'), ('action_count', 'i4')])
    np.savetxt(
        planning_time_csv,
        planning_time_array,
        delimiter=',',
        header='mode,nodes,time,action_count',
        comments='',
        fmt='%s,%d,%.18e,%d')


def plot_results(folder_name, planning_time_list, modes, show_plot):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    for mode in modes:
        mode_nodes = np.array(
            [nodes for m, nodes, _, _ in planning_time_list if m == mode], dtype='i4')
        mode_times = np.array(
            [t for m, _, t, _ in planning_time_list if m == mode])
        mode_counts = np.array(
            [ac for m, _, _, ac in planning_time_list if m == mode], dtype='i4')

        unique_nodes, indices = np.unique(mode_nodes, return_inverse=True)
        mean_times = np.bincount(indices, weights=mode_times) / np.bincount(indices)
        sum_sq = np.bincount(indices, weights=(mode_times - mean_times[indices]) ** 2)
        std_times = np.sqrt(sum_sq / np.bincount(indices))

        ax1.plot(unique_nodes, mean_times, marker='o', linestyle='-', label=MODE_LABELS[mode])
        ax1.fill_between(unique_nodes, mean_times - std_times, mean_times + std_times, alpha=0.2)

        valid = mode_counts != NO_PLAN
        if valid.any():
            v_nodes = mode_nodes[valid]
            v_counts = mode_counts[valid].astype(float)
            u_nodes, idx = np.unique(v_nodes, return_inverse=True)
            mean_ac = np.bincount(idx, weights=v_counts) / np.bincount(idx)
            sum_sq_ac = np.bincount(idx, weights=(v_counts - mean_ac[idx]) ** 2)
            std_ac = np.sqrt(sum_sq_ac / np.bincount(idx))
            ax2.plot(u_nodes, mean_ac, marker='o', linestyle='-', label=MODE_LABELS[mode])
            ax2.fill_between(u_nodes, mean_ac - std_ac, mean_ac + std_ac, alpha=0.2)

    ax1.set_xlabel('Number of Nodes')
    ax1.set_ylabel('Mean Elapsed Time (seconds)')
    ax1.set_ylim(bottom=0)
    ax1.set_title('Average Planning Execution Time with Std Dev')
    ax1.legend()
    ax1.grid(True)

    ax2.set_xlabel('Number of Nodes')
    ax2.set_ylabel('Mean Plan Length (actions)')
    ax2.set_ylim(bottom=0)
    ax2.set_title('Average Plan Length with Std Dev')
    ax2.legend()
    ax2.grid(True)

    planning_time_png = folder_name / 'planning_time_avg_std.png'
    plt.savefig(planning_time_png, format='png', dpi=300, bbox_inches='tight')

    if show_plot:
        plt.show(block=True)
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run the navigation grid-map scalability experiment.')
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
        '--runs',
        type=int,
        default=10,
        help='Number of runs for each node count and mode.')
    parser.add_argument(
        '--show-plot',
        action='store_true',
        help='Display the plot window after saving it.')
    parser.add_argument(
        '--search',
        default='astar(blind())',
        help='Fast Downward search configuration string.')
    return parser.parse_args()


def runner(
        mode: str = 'adaptive',
        runs: int = 10,
        show_plot: bool = False,
        search: str = 'astar(blind())',
        out_dir: Path | None = None) -> Path:
    min_nodes = 10
    max_nodes = 1000
    nodes_interval = 10

    nodes_skip = 0.1
    unconnected_amount = 0.15
    unsafe_amount = 0.25
    dark_amount = 0.25

    modes = ['adaptive', 'no-adaptation'] if mode == 'both' else [mode]
    planning_time_list = []

    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder_name = out_dir if out_dir is not None else (REPO_ROOT / 'results' / 'grid_map' / date)
    folder_name.mkdir(parents=True, exist_ok=True)

    for current_mode in modes:
        print(f'\n--- {MODE_LABELS[current_mode]} ---')
        for n_nodes in range(
                min_nodes,
                max_nodes + nodes_interval,
                nodes_interval):
            for run_n in range(runs):
                result = run_single_case(
                    folder_name,
                    current_mode,
                    run_n,
                    n_nodes,
                    nodes_skip,
                    unconnected_amount,
                    unsafe_amount,
                    dark_amount,
                    search)
                planning_time_list.append(result)

    folder_mode = folder_name / mode
    csv_path = folder_name / 'planning_times.csv'
    save_results(folder_mode, planning_time_list)
    print(f'\nResults saved to {csv_path}')
    plot_results(folder_mode, planning_time_list, modes, show_plot)
    return csv_path


if __name__ == '__main__':
    _args = parse_args()
    runner(mode=_args.mode, runs=_args.runs, show_plot=_args.show_plot, search=_args.search)
