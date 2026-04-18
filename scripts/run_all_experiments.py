#!/usr/bin/env python3

"""Run all navigation_planta experiments sequentially.

New scalability experiments (Exp A–D) accept --runs N.
Legacy scripts (grid map, Camara PDDL/PRISM) have hardcoded run counts.

Usage:
    python scripts/run_all_experiments.py [--runs N] [--skip <name> ...]

    --runs N      Number of runs per condition for scalability experiments (default: 10).
    --skip        Space-separated experiment names to skip. Valid names:
                      grid_map  camara_pddl  camara_prism  camara_discretized
                      fd  ma  corridor  combined
Examples:
    python scripts/run_all_experiments.py
    python scripts/run_all_experiments.py --runs 5
    python scripts/run_all_experiments.py --skip camara_prism combined
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent


def _run(label: str, cmd: list[str]) -> bool:
    """Run a subprocess, stream its output, and return True on success."""
    print(f'\n{"=" * 60}')
    print(f'  {label}')
    print(f'  Command: {" ".join(str(c) for c in cmd)}')
    print(f'{"=" * 60}')
    t0 = time.perf_counter()
    result = subprocess.run(cmd, cwd=SCRIPT_DIR.parent)
    elapsed = time.perf_counter() - t0
    if result.returncode == 0:
        print(f'  [OK] {label} finished in {elapsed:.1f}s')
    else:
        print(f'  [FAILED] {label} exited with code {result.returncode} after {elapsed:.1f}s')
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(
        description='Run all navigation_planta experiments sequentially.')
    parser.add_argument(
        '--runs', type=int, default=10, metavar='N',
        help='Runs per condition for scalability experiments (default: 10)')
    parser.add_argument(
        '--skip', nargs='*', default=[], metavar='NAME',
        help='Experiment names to skip (grid_map camara_pddl camara_prism '
             'camara_discretized fd ma corridor combined)')
    parser.add_argument(
        '--search', default='lazy_greedy([ff()], preferred=[ff()])',
        help='Fast Downward search configuration string passed to all PDDL experiments.')
    args = parser.parse_args()

    skip = set(args.skip)
    py = sys.executable
    runs = str(args.runs)
    search_args = ['--search', args.search]

    experiments = [
        # (name, label, command)
        ('grid_map',
         'Grid map scalability (vary nodes, PDDL)',
         [py, str(SCRIPT_DIR / 'run_grid_map_scenario.py')] + search_args),

        ('camara_pddl',
         'Camara 2020 map — PDDL (PLANTA)',
         [py, str(SCRIPT_DIR / 'run_camara_scenario.py')] + search_args),

        ('camara_prism',
         'Camara 2020 map — PRISM (baseline)',
         [py, str(SCRIPT_DIR / 'run_camara_prism_scenario.py')]),

        ('camara_discretized',
         'Camara 2020 map — discretized PDDL',
         [py, str(SCRIPT_DIR / 'run_camara_scenario_discretized.py')] + search_args),

        ('fd',
         'Exp A — FD scaling (vary FunctionDesigns N={2,4,6,8,10,12,15})',
         [py, str(SCRIPT_DIR / 'run_fd_scale_scenario.py'), '--runs', runs] + search_args),

        ('corridor',
         'Exp B — Corridor type scaling (vary QA dimensions)',
         [py, str(SCRIPT_DIR / 'run_corridor_type_scale_scenario.py'), '--runs', runs] + search_args),

        ('ma',
         'Exp C — Mission action type scaling (vary M={1..5})',
         [py, str(SCRIPT_DIR / 'run_mission_action_scale_scenario.py'), '--runs', runs] + search_args),

        ('combined',
         'Exp D — Combined scalability (vary nodes + FDs)',
         [py, str(SCRIPT_DIR / 'run_combined_scenario.py'), '--runs', runs] + search_args),
    ]

    results = {}
    total_start = time.perf_counter()

    for name, label, cmd in experiments:
        if name in skip:
            print(f'\n[SKIPPED] {label}')
            results[name] = 'skipped'
            continue
        success = _run(label, cmd)
        results[name] = 'ok' if success else 'FAILED'

    total_elapsed = time.perf_counter() - total_start

    print(f'\n{"=" * 60}')
    print('  Summary')
    print(f'{"=" * 60}')
    for name, label, _ in experiments:
        status = results.get(name, '?')
        print(f'  {status:8s}  {label}')
    print(f'\n  Total wall time: {total_elapsed / 60:.1f} min')


if __name__ == '__main__':
    main()
