#!/usr/bin/env python3

"""Experiment B — Corridor type scaling with optional no-adaptation mode."""

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
from navigation_planta.scenario_variants import CorridorTypeMapGenerator

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


def make_generator(mode: str, k: int) -> MapGenerator:
    if mode == 'no-adaptation':
        return NoAdaptationMapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT)
    if k == 2:
        return MapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT)
    if k == 3:
        return CorridorTypeMapGenerator(
            N_NODES, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT,
            outdoor_amount=OUTDOOR_AMOUNT)
    if k == 4:
        return CorridorTypeMapGenerator(
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


def run_single(folder: Path, mode: str, run_id: int, k: int, search: str = 'lazy_greedy([ff()], preferred=[ff()])', seed: int | None = None):
    """Run one experiment trial."""
    def prepare_problem(run_folder: Path, current_mode: str) -> Path:
        if seed is not None:
            import random
            random.seed(seed)
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
    plot_numeric_sweep_results(
        folder,
        records,
        {mode: MODE_LABELS[mode] for mode in modes},
        xlabel='Number of Corridor Types (K)',
        time_title='Planning Time vs. Number of Corridor Types',
        action_title='Plan Length vs. Number of Corridor Types',
        filename='planning_time_ct_scale.png',
        x_values=K_VALUES,
        xticks=K_VALUES,
    )


def runner(n_runs: int, mode: str, search: str = 'lazy_greedy([ff()], preferred=[ff()])', out_dir: Path | None = None, base_seed: int | None = None) -> Path:
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
        after_run=lambda folder_mode, records, _modes: plot_memory_summary(
            folder_mode, records, MODE_LABELS, filename='peak_memory_boxplot.png'),
        value_printer=lambda k: f'  K = {k}',
        search=search,
        out_dir=out_dir,
        base_seed=base_seed,
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
