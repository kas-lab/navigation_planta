#!/usr/bin/env python3

"""Experiment A — FunctionDesign scaling with optional no-adaptation mode."""

import argparse
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

from navigation_planta.experiment_runner import (
    SweepExperimentConfig,
    execute_pddl_case,
    run_sweep_experiment,
)
from navigation_planta.map_generator import MapGenerator
from navigation_planta.no_adaptation import NoAdaptationMapGenerator
from navigation_planta.reporting import plot_memory_summary, plot_numeric_sweep_results

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

    def prepare_problem(run_folder: Path, current_mode: str) -> Path:
        mg = make_generator(current_mode)
        mg.generate_connected_grid_map()

        problem_file = run_folder / (
            'problem.pddl' if current_mode == 'adaptive' else 'problem_no_sas.pddl')
        mg.generate_domain_problem_files(save_problem=True, problem_filename=problem_file)
        mg.plot_graph(show_plot=False, save_file=True, filename=run_folder / 'map.png')
        return problem_file

    record = execute_pddl_case(
        folder=folder,
        mode=mode,
        case_id=f'fd{n_fd}_run{run_id}',
        x_value=n_fd,
        search=search,
        prepare_problem=prepare_problem,
        adaptive_preprocessor=lambda problem_file, run_folder: run_adaptive_preprocessor(
            OWL_FILES[n_fd],
            problem_file,
            run_folder,
        ),
        baseline_domain=BASELINE_DOMAIN_FILE,
        include_owltopddl_time=True,
    )

    print(
        f'mode={mode:<13} n_fd={n_fd:2d} run={run_id:2d} '
        f'owltopddl={record.owltopddl_time:.3f}s planning={record.planning_time:.6f}s '
        f'actions={record.action_count}'
    )
    return record

def plot_results(folder: Path, records, modes):
    plot_numeric_sweep_results(
        folder,
        records,
        {mode: MODE_LABELS[mode] for mode in modes},
        xlabel='Number of FunctionDesigns (N)',
        time_title='Planning Time vs. Number of FunctionDesigns',
        action_title='Plan Length vs. Number of FunctionDesigns',
        filename='planning_time_fd_scale.png',
        x_values=N_FD_VALUES,
    )


def runner(n_runs: int, mode: str, search: str = 'lazy_greedy([ff()], preferred=[ff()])', out_dir: Path | None = None) -> Path:
    config = SweepExperimentConfig(
        results_root=REPO_ROOT / 'results' / 'scalability_fd',
        mode_labels=MODE_LABELS,
        x_values=N_FD_VALUES,
        x_name='n_fd',
        time_name='planning_time',
    )
    return run_sweep_experiment(
        config,
        n_runs=n_runs,
        mode=mode,
        run_one=run_single,
        plot_results=plot_results,
        after_run=lambda folder_mode, records, _modes: plot_memory_summary(
            folder_mode, records, MODE_LABELS, filename='peak_memory_boxplot.png'),
        value_printer=lambda n_fd: f'  N_FD = {n_fd}',
        search=search,
        out_dir=out_dir,
    )


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
        '--search', default='astar(blind())',
        help='Fast Downward search configuration string (default: astar blind)')
    args = parser.parse_args()

    print(
        f'FD scaling experiment: mode={args.mode}, N_FD={N_FD_VALUES}, '
        f'runs={args.runs}, nodes={N_NODES}'
    )
    runner(args.runs, args.mode, args.search)


if __name__ == '__main__':
    main()
