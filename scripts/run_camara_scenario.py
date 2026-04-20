#!/usr/bin/env python3

"""Cámara navigation experiment runner with optional no-adaptation baseline."""

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
from navigation_planta.reporting import (
    plot_mode_summary,
    summarize_mode_records,
    write_summary_report,
)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

MAP_FILE = REPO_ROOT / 'data' / 'map_camara_2020_paper' / 'map-p2cp3.json'
RESULTS_ROOT = REPO_ROOT / 'results' / 'map_camara_2020_paper'
OWL_FILE = REPO_ROOT / 'owl' / 'navigation.owl'
ADAPTIVE_DOMAIN_FILE = REPO_ROOT / 'pddl' / 'domain_sas.pddl'
BASELINE_DOMAIN_FILE = REPO_ROOT / 'pddl' / 'domain_no_sas.pddl'

MODE_LABELS = {
    'adaptive': 'Adaptive',
    'no-adaptation': 'No adaptation',
}

UNREACHABLE_NODES = [20, 41]


def make_map_generator(mode):
    generator_class = MapGenerator if mode == 'adaptive' else NoAdaptationMapGenerator
    map_generator = generator_class()
    map_generator.load_json(MAP_FILE)
    return map_generator


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


def run_single_case(folder_name, mode, run_n, init_goal, search='lazy_greedy([ff()], preferred=[ff()])'):
    init, goal = init_goal
    map_folder_name = f'wp{init}_wp{goal}_{run_n}'

    def prepare_problem(run_folder: Path, current_mode: str) -> Path:
        map_generator = make_map_generator(current_mode)
        problem_filename = run_folder / (
            'problem.pddl' if current_mode == 'adaptive' else 'problem_no_sas.pddl')
        map_generator.generate_domain_problem_files(
            save_problem=True,
            problem_filename=problem_filename,
            init_goal=(init, goal))
        return problem_filename

    record = execute_pddl_case(
        folder=folder_name,
        mode=mode,
        case_id=map_folder_name,
        x_value=f'wp{init}_wp{goal}',
        search=search,
        prepare_problem=prepare_problem,
        adaptive_preprocessor=run_adaptive_preprocessor,
        baseline_domain=BASELINE_DOMAIN_FILE,
    )

    print(
        f'mode={mode:<13} init={init:<2} goal={goal:<2} '
        f'run={run_n:<2} time={record.planning_time:.6f}s '
        f'actions={record.action_count} mem={record.peak_memory:.1f}MB')
    return record

def plot_results(folder_name, planning_time_list, modes):
    plot_mode_summary(
        folder_name, planning_time_list,
        {m: MODE_LABELS[m] for m in modes},
        filename='camara_summary.png')


def print_summary(folder_name: Path, planning_time_list, modes):
    summary_lines = summarize_mode_records(
        planning_time_list,
        {m: MODE_LABELS[m] for m in modes},
    )
    for line in summary_lines:
        print(line)
    write_summary_report(folder_name, summary_lines)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run the navigation Cámara experiment.')
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
        default=1,
        help='Number of runs for each init/goal pair and mode.')
    parser.add_argument(
        '--search',
        default='lazy_greedy([ff()], preferred=[ff()])',
        help='Fast Downward search configuration string.')
    return parser.parse_args()


def runner(
        mode: str = 'adaptive',
        runs: int = 1,
        search: str = 'lazy_greedy([ff()], preferred=[ff()])',
        out_dir: Path | None = None) -> Path:
    init_goal_pairs = [
        (init, goal)
        for init in range(1, 59)
        for goal in range(1, 59)
        if init != goal and init not in UNREACHABLE_NODES and goal not in UNREACHABLE_NODES
    ]
    config = SweepExperimentConfig(
        results_root=RESULTS_ROOT,
        mode_labels=MODE_LABELS,
        x_values=init_goal_pairs,
        x_name='init_goal',
        time_name='time',
        per_mode_results_folder=False,
    )
    return run_sweep_experiment(
        config,
        n_runs=runs,
        mode=mode,
        run_one=run_single_case,
        plot_results=plot_results,
        after_run=lambda folder_name, records, modes: print_summary(folder_name, records, modes),
        search=search,
        out_dir=out_dir,
    )


if __name__ == '__main__':
    _args = parse_args()
    runner(mode=_args.mode, runs=_args.runs, search=_args.search)
