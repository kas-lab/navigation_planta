#!/usr/bin/env python3

"""Cámara navigation experiment runner with optional no-adaptation baseline."""

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


def run_single_case(folder_name, mode, run_n, init, goal, search='lazy_greedy([ff()], preferred=[ff()])'):
    map_generator = make_map_generator(mode)

    map_folder_name = f'wp{init}_wp{goal}_{run_n}'
    run_folder = folder_name / mode / map_folder_name
    run_folder.mkdir(parents=True, exist_ok=True)

    problem_filename = run_folder / (
        'problem.pddl' if mode == 'adaptive' else 'problem_no_sas.pddl')
    map_generator.generate_domain_problem_files(
        save_problem=True,
        problem_filename=problem_filename,
        init_goal=(init, goal))

    if mode == 'adaptive':
        try:
            domain_for_planner, problem_for_planner = run_adaptive_preprocessor(
                problem_filename,
                run_folder)
        except subprocess.CalledProcessError as error:
            print(
                f'OWLToPDDL failed for mode={mode} init={init} goal={goal} '
                f'run={run_n}: {error.stderr}')
            return run_single_case(folder_name, mode, run_n, init, goal, search)
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
        f'mode={mode:<13} init={init:<2} goal={goal:<2} '
        f'run={run_n:<2} time={elapsed_time:.6f}s actions={action_count}')
    return mode, f'wp{init}_wp{goal}', elapsed_time, action_count


def save_results(folder_name, planning_time_list):
    planning_time_csv = folder_name / 'planning_times.csv'
    single_mode = len({mode for mode, _, _, _ in planning_time_list}) == 1

    if single_mode:
        planning_time_array = np.array(
            [(init_goal, elapsed_time, action_count)
             for _, init_goal, elapsed_time, action_count in planning_time_list],
            dtype=[('init_goal', 'U15'), ('time', 'f8'), ('action_count', 'i4')])
        np.savetxt(
            planning_time_csv,
            planning_time_array,
            delimiter=',',
            header='init_goal,time,action_count',
            comments='',
            fmt='%s,%.18e,%d')
        return

    planning_time_array = np.array(
        [(mode, init_goal, elapsed_time, action_count)
         for mode, init_goal, elapsed_time, action_count in planning_time_list],
        dtype=[('mode', 'U20'), ('init_goal', 'U15'), ('time', 'f8'), ('action_count', 'i4')])
    np.savetxt(
        planning_time_csv,
        planning_time_array,
        delimiter=',',
        header='mode,init_goal,time,action_count',
        comments='',
        fmt='%s,%s,%.18e,%d')


def plot_results(folder_name, planning_time_list, modes):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    labels = [MODE_LABELS[m] for m in modes]
    mean_times = []
    std_times = []
    mean_actions = []
    std_actions = []

    for mode in modes:
        times = np.array([t for m, _, t, _ in planning_time_list if m == mode])
        counts = np.array(
            [ac for m, _, _, ac in planning_time_list if m == mode and ac != NO_PLAN],
            dtype=float)
        mean_times.append(times.mean())
        std_times.append(times.std())
        mean_actions.append(counts.mean() if counts.size else float('nan'))
        std_actions.append(counts.std() if counts.size else 0.0)

    x = np.arange(len(modes))
    ax1.bar(x, mean_times, yerr=std_times, capsize=5, color='steelblue', alpha=0.8)
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels)
    ax1.set_ylabel('Mean Planning Time (seconds)')
    ax1.set_title('Planning Time by Mode')
    ax1.set_ylim(bottom=0)
    ax1.grid(True, axis='y')

    ax2.bar(x, mean_actions, yerr=std_actions, capsize=5, color='darkorange', alpha=0.8)
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels)
    ax2.set_ylabel('Mean Plan Length (actions)')
    ax2.set_title('Plan Length by Mode')
    ax2.set_ylim(bottom=0)
    ax2.grid(True, axis='y')

    plot_path = folder_name / 'camara_summary.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def print_summary(planning_time_list, modes):
    if len(modes) == 1:
        times = np.array([elapsed_time for _, _, elapsed_time, _ in planning_time_list])
        counts = np.array([ac for _, _, _, ac in planning_time_list if ac != NO_PLAN], dtype=float)
        print(f'Mean {times.mean():.4f}s Std dev: {times.std():.4f}s '
              f'Mean actions: {counts.mean():.1f}' if counts.size else f'Mean {times.mean():.4f}s')
        return

    for mode in modes:
        times = np.array([
            t for m, _, t, _ in planning_time_list if m == mode
        ])
        counts = np.array([
            ac for m, _, _, ac in planning_time_list if m == mode and ac != NO_PLAN
        ], dtype=float)
        ac_str = f' mean actions: {counts.mean():.1f}' if counts.size else ''
        print(f'{MODE_LABELS[mode]} mean {times.mean():.4f}s std dev: {times.std():.4f}s{ac_str}')


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
    planning_time_list = []
    modes = ['adaptive', 'no-adaptation'] if mode == 'both' else [mode]

    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder_name = out_dir if out_dir is not None else (RESULTS_ROOT / date)
    folder_name.mkdir(parents=True, exist_ok=True)

    for current_mode in modes:
        print(f'\n--- {MODE_LABELS[current_mode]} ---')
        for init in range(1, 59):
            for goal in range(1, 59):
                if (
                        init != goal and
                        init not in UNREACHABLE_NODES and
                        goal not in UNREACHABLE_NODES):
                    for run_n in range(runs):
                        result = run_single_case(folder_name, current_mode, run_n, init, goal, search)
                        planning_time_list.append(result)

    save_results(folder_name, planning_time_list)
    plot_results(folder_name, planning_time_list, modes)
    print_summary(planning_time_list, modes)
    return folder_name / 'planning_times.csv'


if __name__ == '__main__':
    _args = parse_args()
    runner(mode=_args.mode, runs=_args.runs, search=_args.search)
