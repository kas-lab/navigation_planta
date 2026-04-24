#!/usr/bin/env python3

"""Experiment D — Combined stress test with optional no-adaptation mode."""

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
from navigation_planta.reporting import plot_memory_summary, plot_numeric_sweep_results
from navigation_planta.scenario_variants import (
    CombinedScenarioMapGenerator,
    MissionActionNoAdaptationMapGenerator,
)

OWL_FILE = REPO_ROOT / 'owl' / 'experiment_combined_scalability' / 'navigation_combined.owl'
ADAPTIVE_DOMAIN_FILE = (
    REPO_ROOT / 'pddl' / 'experiment_combined_scalability' / 'domain_sas_combined.pddl'
)
BASELINE_DOMAIN_FILE = (
    REPO_ROOT / 'pddl' / 'experiment_combined_scalability' / 'domain_no_sas_combined.pddl'
)

MODE_LABELS = {
    'adaptive': 'Adaptive',
    'no-adaptation': 'No adaptation',
}

MIN_NODES = 10
MAX_NODES = 100
NODES_STEP = 10
NODES_SKIP = 0.1
UNCONNECTED_AMOUNT = 0.15
UNSAFE_AMOUNT = 0.25
DARK_AMOUNT = 0.25
OUTDOOR_AMOUNT = 0.20
DUSTY_AMOUNT = 0.20


def make_generator(mode: str, n_nodes: int) -> MapGenerator:
    if mode == 'no-adaptation':
        return MissionActionNoAdaptationMapGenerator(
            n_nodes, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT, m=5)
    return CombinedScenarioMapGenerator(
        n_nodes, NODES_SKIP, UNCONNECTED_AMOUNT, UNSAFE_AMOUNT, DARK_AMOUNT)


def run_adaptive_preprocessor(problem_file, run_folder):
    domain_out = run_folder / 'domain_created.pddl'
    problem_out = run_folder / 'problem_created.pddl'
    subprocess.run(
        [
            'OWLToPDDL.sh',
            f'--owl={OWL_FILE}',
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


def run_single(folder: Path, mode: str, run_id: int, n_nodes: int, search: str = 'astar(blind())', seed: int | None = None):
    """Run one trial for a given map size."""
    n_nodes_resulting = n_nodes - int(n_nodes * NODES_SKIP)
    def prepare_problem(run_folder: Path, current_mode: str) -> Path:
        if seed is not None:
            import random
            random.seed(seed)
        mg = make_generator(current_mode, n_nodes)
        mg.generate_connected_grid_map()

        problem_file = run_folder / (
            'problem.pddl' if current_mode == 'adaptive' else 'problem_no_sas.pddl')
        mg.generate_domain_problem_files(save_problem=True, problem_filename=problem_file)
        mg.plot_graph(show_plot=False, save_file=True, filename=run_folder / 'map.png')
        return problem_file

    record = execute_pddl_case(
        folder=folder,
        mode=mode,
        case_id=f'n{n_nodes}_run{run_id}',
        x_value=n_nodes_resulting,
        search=search,
        prepare_problem=prepare_problem,
        adaptive_preprocessor=run_adaptive_preprocessor,
        baseline_domain=BASELINE_DOMAIN_FILE,
        include_owltopddl_time=True,
    )

    print(
        f'mode={mode:<13} n={n_nodes:4d} (eff={n_nodes_resulting:3d}) run={run_id:2d} '
        f'owltopddl={record.owltopddl_time:.3f}s planning={record.planning_time:.6f}s '
        f'actions={record.action_count} peak_memory={record.peak_memory:.2f}MB'
    )
    return record

def plot_results(folder: Path, records, modes):
    plot_numeric_sweep_results(
        folder,
        records,
        {mode: MODE_LABELS[mode] for mode in modes},
        xlabel='Effective Map Size (nodes)',
        time_title='Combined Stress Test: Planning Time vs. Map Size',
        action_title='Combined Stress Test: Plan Length vs. Map Size',
        filename='planning_time_combined.png',
        x_values=sorted({record.x_value for record in records}),
        figsize=(16, 5),
        markersize=3,
    )


def runner(n_runs: int, mode: str, min_nodes: int = MIN_NODES, max_nodes: int = MAX_NODES, search: str = 'astar(blind())', out_dir: Path | None = None, base_seed: int | None = None) -> Path:
    config = SweepExperimentConfig(
        results_root=REPO_ROOT / 'results' / 'scalability_combined',
        mode_labels=MODE_LABELS,
        x_values=list(range(min_nodes, max_nodes + NODES_STEP, NODES_STEP)),
        x_name='nodes',
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
        search=search,
        out_dir=out_dir,
        base_seed=base_seed,
    )


def main():
    parser = argparse.ArgumentParser(
        description='Experiment D: Combined stress test — maximum complexity, vary map size.')
    parser.add_argument(
        '--mode',
        choices=['adaptive', 'no-adaptation', 'both'],
        default='adaptive',
        help=(
            'Experiment variant to run. '
            '"adaptive" reproduces the current PLANTA flow, '
            '"no-adaptation" uses the combined no-adaptation domain directly, '
            'and "both" runs both variants for comparison.'
        ),
    )
    parser.add_argument(
        '--runs', type=int, default=10, metavar='N',
        help='Number of runs per map size (default: 10)')
    parser.add_argument(
        '--min-nodes', type=int, default=MIN_NODES, metavar='N',
        help=f'Minimum map size in nodes (default: {MIN_NODES})')
    parser.add_argument(
        '--max-nodes', type=int, default=MAX_NODES, metavar='N',
        help=f'Maximum map size in nodes (default: {MAX_NODES})')
    parser.add_argument(
        '--search', default='astar(blind())',
        help='Fast Downward search configuration string.')
    args = parser.parse_args()

    min_nodes = args.min_nodes
    max_nodes = args.max_nodes
    node_count = len(range(min_nodes, max_nodes + NODES_STEP, NODES_STEP))
    print(
        f'Combined stress test: mode={args.mode}, {node_count} map sizes '
        f'({min_nodes}–{max_nodes}, step {NODES_STEP}) × {args.runs} runs'
    )
    runner(args.runs, args.mode, min_nodes=min_nodes, max_nodes=max_nodes, search=args.search)


if __name__ == '__main__':
    main()
