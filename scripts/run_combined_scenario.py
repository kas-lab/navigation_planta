#!/usr/bin/env python3

"""Experiment D — Combined stress test with optional no-adaptation mode."""

import argparse
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

from navigation_planta.experiment_runner import (
    SweepExperimentConfig,
    execute_pddl_case,
    run_sweep_experiment,
)
from navigation_planta.map_generator import MapGenerator
from navigation_planta.scenario_variants import (
    CombinedScenarioMapGenerator,
    MissionActionNoAdaptationMapGenerator,
)
from navigation_planta.utils import (
    NO_PLAN,
    plot_memory_boxplot,
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


def run_single(folder: Path, mode: str, run_id: int, n_nodes: int, search: str = 'astar(blind())'):
    """Run one trial for a given map size."""
    n_nodes_resulting = n_nodes - int(n_nodes * NODES_SKIP)
    def prepare_problem(run_folder: Path, current_mode: str) -> Path:
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
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5))

    unique_nodes = sorted({record.x_value for record in records})
    for mode in modes:
        mean_t, std_t, mean_ac, std_ac = [], [], [], []
        for node_count in unique_nodes:
            times = np.array([
                record.planning_time
                for record in records
                if record.mode == mode and record.x_value == node_count
            ])
            mean_t.append(times.mean())
            std_t.append(times.std())
            counts = np.array([
                record.action_count
                for record in records
                if record.mode == mode and record.x_value == node_count and record.action_count != NO_PLAN
            ], dtype=float)
            mean_ac.append(counts.mean() if counts.size else float('nan'))
            std_ac.append(counts.std() if counts.size else 0.0)

        mean_t, std_t = np.array(mean_t), np.array(std_t)
        mean_ac, std_ac = np.array(mean_ac), np.array(std_ac)

        ax1.plot(unique_nodes, mean_t, marker='o', linestyle='-', markersize=3,
                 label=MODE_LABELS[mode])
        ax1.fill_between(unique_nodes, mean_t - std_t, mean_t + std_t, alpha=0.2)
        ax2.plot(unique_nodes, mean_ac, marker='o', linestyle='-', markersize=3,
                 label=MODE_LABELS[mode])
        ax2.fill_between(unique_nodes, mean_ac - std_ac, mean_ac + std_ac, alpha=0.2)

    ax1.set_xlabel('Effective Map Size (nodes)')
    ax1.set_ylabel('Mean Planning Time (seconds)')
    ax1.set_title('Combined Stress Test: Planning Time vs. Map Size')
    ax1.legend()
    ax1.grid(True)

    ax2.set_xlabel('Effective Map Size (nodes)')
    ax2.set_ylabel('Mean Plan Length (actions)')
    ax2.set_ylim(bottom=0)
    ax2.set_title('Combined Stress Test: Plan Length vs. Map Size')
    ax2.legend()
    ax2.grid(True)

    plot_path = folder / 'planning_time_combined.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def runner(n_runs: int, mode: str, min_nodes: int = MIN_NODES, max_nodes: int = MAX_NODES, search: str = 'astar(blind())', out_dir: Path | None = None) -> Path:
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
        after_run=lambda folder_mode, records, _modes: plot_memory_boxplot(
            folder_mode, records, MODE_LABELS, filename='peak_memory_boxplot.png'),
        search=search,
        out_dir=out_dir,
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
