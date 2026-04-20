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
from pathlib import Path

from navigation_planta.experiment_runner import (
    SweepExperimentConfig,
    execute_pddl_case,
    run_sweep_experiment,
)
from navigation_planta.map_generator import MapGenerator
from navigation_planta.no_adaptation import NoAdaptationMapGenerator
from navigation_planta.reporting import plot_memory_summary, plot_numeric_sweep_results

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
    map_folder_name = (
        f'{n_nodes}_{nodes_skip}_{unconnected_amount}_{unsafe_amount}_{dark_amount}_{run_n}'
    )

    def prepare_problem(run_folder: Path, current_mode: str) -> Path:
        map_generator = make_map_generator(
            current_mode,
            n_nodes,
            nodes_skip,
            unconnected_amount,
            unsafe_amount,
            dark_amount)
        map_generator.generate_connected_grid_map()

        problem_filename = run_folder / (
            'problem.pddl' if current_mode == 'adaptive' else 'problem_no_sas.pddl')
        map_generator.generate_domain_problem_files(
            save_problem=True,
            problem_filename=problem_filename)
        map_generator.plot_graph(
            show_plot=False,
            save_file=True,
            filename=run_folder / 'map.png')
        return problem_filename

    record = execute_pddl_case(
        folder=folder_name,
        mode=mode,
        case_id=map_folder_name,
        x_value=n_nodes_resulting,
        search=search,
        prepare_problem=prepare_problem,
        adaptive_preprocessor=run_adaptive_preprocessor,
        baseline_domain=BASELINE_DOMAIN_FILE,
    )

    print(
        f'mode={mode:<13} nodes={n_nodes_resulting:<4} '
        f'run={run_n:<2} time={record.planning_time:.6f}s '
        f'actions={record.action_count}')
    return record

def plot_results(folder_name, planning_time_list, modes, show_plot):
    plot_numeric_sweep_results(
        folder_name,
        planning_time_list,
        {mode: MODE_LABELS[mode] for mode in modes},
        xlabel='Number of Nodes',
        time_title='Average Planning Execution Time with Std Dev',
        action_title='Average Plan Length with Std Dev',
        filename='planning_time_avg_std.png',
        x_values=sorted({record.x_value for record in planning_time_list}),
        time_ylabel='Mean Elapsed Time (seconds)',
        show_plot=show_plot,
    )


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

    config = SweepExperimentConfig(
        results_root=REPO_ROOT / 'results' / 'grid_map',
        mode_labels=MODE_LABELS,
        x_values=list(range(min_nodes, max_nodes + nodes_interval, nodes_interval)),
        x_name='nodes',
        time_name='time',
    )
    return run_sweep_experiment(
        config,
        n_runs=runs,
        mode=mode,
        run_one=lambda folder_name, current_mode, run_n, n_nodes, search: run_single_case(
            folder_name,
            current_mode,
            run_n,
            n_nodes,
            nodes_skip,
            unconnected_amount,
            unsafe_amount,
            dark_amount,
            search,
        ),
        plot_results=lambda folder_name, records, modes: plot_results(
            folder_name, records, modes, show_plot),
        after_run=lambda folder_name, records, _modes: plot_memory_summary(
            folder_name, records, MODE_LABELS, filename='peak_memory_boxplot.png'),
        search=search,
        out_dir=out_dir,
    )


if __name__ == '__main__':
    _args = parse_args()
    runner(mode=_args.mode, runs=_args.runs, show_plot=_args.show_plot, search=_args.search)
