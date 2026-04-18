#!/usr/bin/env python3

"""Cámara navigation experiment runner with optional no-adaptation baseline."""

import argparse
import subprocess
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from map_generator import MapGenerator
from no_adaptation import NoAdaptationMapGenerator

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

MAP_FILE = REPO_ROOT / 'map_camara_2020_paper' / 'map-p2cp3.json'
RESULTS_ROOT = REPO_ROOT / 'map_camara_2020_paper'
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

    start_time = time.perf_counter()
    subprocess.run([
        'fast-downward.py',
        str(domain_for_planner),
        str(problem_for_planner),
        '--search', search,
    ], check=True)
    elapsed_time = time.perf_counter() - start_time

    print(
        f'mode={mode:<13} init={init:<2} goal={goal:<2} '
        f'run={run_n:<2} time={elapsed_time:.6f}s')
    return mode, f'wp{init}_wp{goal}', elapsed_time


def save_results(folder_name, planning_time_list):
    planning_time_csv = folder_name / 'planning_times.csv'
    single_mode = len({mode for mode, _, _ in planning_time_list}) == 1

    if single_mode:
        planning_time_array = np.array(
            [(init_goal, elapsed_time) for _, init_goal, elapsed_time in planning_time_list],
            dtype=[('init_goal', 'U15'), ('time', 'f8')])
        np.savetxt(
            planning_time_csv,
            planning_time_array,
            delimiter=',',
            header='init_goal,time',
            comments='',
            fmt='%s,%.18e')
        return

    planning_time_array = np.array(
        planning_time_list,
        dtype=[('mode', 'U20'), ('init_goal', 'U15'), ('time', 'f8')])
    np.savetxt(
        planning_time_csv,
        planning_time_array,
        delimiter=',',
        header='mode,init_goal,time',
        comments='',
        fmt='%s,%s,%.18e')


def print_summary(planning_time_list, modes):
    if len(modes) == 1:
        times = np.array([elapsed_time for _, _, elapsed_time in planning_time_list])
        print(f'Mean {times.mean()} and Std dev: {times.std()}')
        return

    for mode in modes:
        times = np.array([
            elapsed_time
            for record_mode, _, elapsed_time in planning_time_list
            if record_mode == mode
        ])
        print(
            f'{MODE_LABELS[mode]} mean {times.mean()} and std dev: {times.std()}')


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


def runner():
    args = parse_args()
    planning_time_list = []
    modes = ['adaptive', 'no-adaptation'] if args.mode == 'both' else [args.mode]

    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    folder_name = RESULTS_ROOT / date
    folder_name.mkdir(parents=True, exist_ok=True)

    for mode in modes:
        print(f'\n--- {MODE_LABELS[mode]} ---')
        for init in range(1, 59):
            for goal in range(1, 59):
                if (
                        init != goal and
                        init not in UNREACHABLE_NODES and
                        goal not in UNREACHABLE_NODES):
                    for run_n in range(args.runs):
                        result = run_single_case(folder_name, mode, run_n, init, goal, args.search)
                        planning_time_list.append(result)

    save_results(folder_name, planning_time_list)
    print_summary(planning_time_list, modes)


if __name__ == '__main__':
    runner()
