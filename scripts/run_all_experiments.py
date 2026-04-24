#!/usr/bin/env python3

"""Run all navigation_planta experiments sequentially.

When multiple --search strategies are given, every PDDL experiment is run
once per strategy.  After all runs a comparison plot is saved per experiment.

Usage:
    python scripts/run_all_experiments.py [--runs N] [--skip <name> ...]
                                          [--search S1 [S2 ...]]

    --runs N      Number of runs per condition for scalability experiments (default: 10).
    --skip        Space-separated experiment names to skip. Valid names:
                      grid_map  camara_pddl  camara_prism  camara_discretized
                      fd  ma  corridor  combined
    --search      One or more Fast Downward search strings (default: lazy_greedy+ff).
                  Multiple values run each experiment once per strategy and
                  produce a comparison plot per experiment afterwards.

Examples:
    python scripts/run_all_experiments.py
    python scripts/run_all_experiments.py --runs 5
    python scripts/run_all_experiments.py --skip camara_prism combined
    python scripts/run_all_experiments.py --search 'astar(blind())' 'eager_greedy([goalcount()])'
"""

import argparse
import re
import sys
import time
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

sys.path.insert(0, str(SCRIPT_DIR))

from run_camara_scenario import runner as run_camara
from run_camara_scenario_discretized import runner as run_camara_discretized
from run_camara_prism_scenario import runner as run_prism
from run_combined_scenario import runner as run_combined
from run_corridor_type_scale_scenario import runner as run_corridor
from run_fd_scale_scenario import runner as run_fd
from run_grid_map_scenario import runner as run_grid_map
from run_mission_action_scale_scenario import runner as run_ma
from navigation_planta.reporting import (
    EXPERIMENT_COMPARISON_CONFIG,
    generate_strategy_comparison_plots,
)

PDDL_EXPERIMENT_NAMES = set(EXPERIMENT_COMPARISON_CONFIG.keys())

# Per-family base seeds. Variants sharing a map topology use the same constant
# so each (x_idx, run_id) pair generates an identical underlying graph across
# all search strategies and across adaptive/no-adaptation pairs.
_SEED_GRID_MAP    = 100_000
_SEED_CAMARA_DISC = 200_000
_SEED_FD          = 300_000
_SEED_CORRIDOR    = 400_000
_SEED_MA          = 500_000
_SEED_COMBINED    = 600_000


def _sanitize(s: str) -> str:
    return re.sub(r'[^a-zA-Z0-9_-]', '_', s)[:60]


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
        '--search', nargs='+',
        default=['astar(blind())','eager_greedy([goalcount()])'],
        metavar='SEARCH',
        help='One or more Fast Downward search strings. Multiple values run '
             'each experiment once per strategy and produce comparison plots.')
    args = parser.parse_args()

    skip = set(args.skip)
    searches: list[str] = args.search
    runs: int = args.runs

    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    base_out = REPO_ROOT / 'results' / 'all_experiments' / date
    comparison_dir = base_out / 'comparison'

    # {exp_name: {strategy_label: csv_path}}
    strategy_results: dict[str, dict[str, Path]] = {
        name: {} for name in PDDL_EXPERIMENT_NAMES}
    last_status: dict[str, str] = {}

    def _run_one(name: str, label: str, fn, out_dir: Path, **kwargs) -> Path | None:
        if name in skip:
            print(f'\n[SKIPPED] {label}')
            last_status[name] = 'skipped'
            return None
        out_dir.mkdir(parents=True, exist_ok=True)
        print(f'\n{"=" * 60}')
        print(f'  {label}')
        print(f'{"=" * 60}')
        t0 = time.perf_counter()
        try:
            csv_path = fn(out_dir=out_dir, **kwargs)
            print(f'  [OK] finished in {time.perf_counter() - t0:.1f}s')
            last_status[name] = 'ok'
            return csv_path
        except Exception as exc:
            print(f'  [FAILED] {exc!r} after {time.perf_counter() - t0:.1f}s')
            last_status[name] = 'FAILED'
            return None

    total_start = time.perf_counter()
    prism_done = False

    for search_str in searches:
        strat_label = search_str
        strat_dir = base_out / _sanitize(search_str)
        strat_dir.mkdir(parents=True, exist_ok=True)
        (strat_dir / 'search.txt').write_text(search_str + '\n', encoding='utf-8')

        print(f'\n{"#" * 60}')
        print(f'  Strategy: {search_str}')
        print(f'{"#" * 60}')

        csv = _run_one('grid_map', 'Grid map scalability (vary nodes, PDDL)',
                       run_grid_map, strat_dir / 'grid_map',
                       mode='adaptive', runs=runs, show_plot=False, search=search_str,
                       base_seed=_SEED_GRID_MAP)
        if csv:
            strategy_results['grid_map'][strat_label] = csv

        csv = _run_one('grid_map_no_sas', 'Grid map scalability no adaptation (vary nodes, PDDL)',
                       run_grid_map, strat_dir / 'grid_map',
                       mode='no-adaptation', runs=runs, show_plot=False, search=search_str,
                       base_seed=_SEED_GRID_MAP)
        if csv:
            strategy_results['grid_map_no_sas'][strat_label] = csv

        # csv = _run_one('camara_pddl', 'Camara 2020 map — PDDL (PLANTA)',
        #                run_camara, strat_dir / 'camara_pddl',
        #                mode='adaptive', runs=runs, search=search_str)
        # if csv:
        #     strategy_results['camara_pddl'][strat_label] = csv

        csv = _run_one('camara_discretized', 'Camara 2020 map — discretized PDDL',
                       run_camara_discretized, strat_dir / 'camara_discretized',
                       mode='adaptive', runs=runs, search=search_str,
                       base_seed=_SEED_CAMARA_DISC)
        if csv:
            strategy_results['camara_discretized'][strat_label] = csv

        csv = _run_one('camara_discretized_no_sas', 'Camara 2020 map — discretized PDDL - no adaptation',
                       run_camara_discretized, strat_dir / 'camara_discretized',
                       mode='no-adaptation', runs=runs, search=search_str,
                       base_seed=_SEED_CAMARA_DISC)
        if csv:
            strategy_results['camara_discretized_no_sas'][strat_label] = csv

        csv = _run_one('fd', 'Exp A — FD scaling (vary FunctionDesigns)',
                       run_fd, strat_dir / 'fd',
                       n_runs=runs, mode='adaptive', search=search_str,
                       base_seed=_SEED_FD)
        if csv:
            strategy_results['fd'][strat_label] = csv

        csv = _run_one('corridor', 'Exp B — Corridor type scaling',
                       run_corridor, strat_dir / 'corridor',
                       n_runs=runs, mode='adaptive', search=search_str,
                       base_seed=_SEED_CORRIDOR)
        if csv:
            strategy_results['corridor'][strat_label] = csv

        csv = _run_one('ma', 'Exp C — Mission action scaling',
                       run_ma, strat_dir / 'ma',
                       n_runs=runs, mode='adaptive', search=search_str,
                       base_seed=_SEED_MA)
        if csv:
            strategy_results['ma'][strat_label] = csv

        csv = _run_one('ma_no_sas', 'Exp C — Mission action scaling - no adaptation',
                       run_ma, strat_dir / 'ma',
                       n_runs=runs, mode='no-adaptation', search=search_str,
                       base_seed=_SEED_MA)
        if csv:
            strategy_results['ma_no_sas'][strat_label] = csv

        csv = _run_one('combined', 'Exp D — Combined scalability',
                       run_combined, strat_dir / 'combined',
                       n_runs=runs, mode='adaptive', search=search_str,
                       base_seed=_SEED_COMBINED)
        if csv:
            strategy_results['combined'][strat_label] = csv

        csv = _run_one('combined_no_sas', 'Exp D — Combined scalability - no-adaptation',
                       run_combined, strat_dir / 'combined',
                       n_runs=runs, mode='no-adaptation', search=search_str,
                       base_seed=_SEED_COMBINED)
        if csv:
            strategy_results['combined_no_sas'][strat_label] = csv

    # PRISM baseline has no --search; run it only once across all strategies
    if not prism_done:
        prism_csv = _run_one(
            'camara_prism', 'Camara 2020 map — PRISM (baseline)',
            run_prism, base_out / 'camara_prism', n_runs=runs)
        if prism_csv:
            strategy_results['camara_prism']['prism'] = prism_csv
            # Inject PRISM as a reference series in the discretized comparisons
            strategy_results['camara_discretized']['prism (baseline)'] = prism_csv
            strategy_results['camara_discretized_no_sas']['prism (baseline)'] = prism_csv
        prism_done = True

    total_elapsed = time.perf_counter() - total_start

    print(f'\n{"=" * 60}')
    print('  Summary')
    print(f'{"=" * 60}')
    for name in PDDL_EXPERIMENT_NAMES:
        status = last_status.get(name, '?')
        print(f'  {status:8s}  {name}')
    print(f'\n  Total wall time: {total_elapsed / 60:.1f} min')
    print(f'  Results in: {base_out}')

    generate_comparisons = len(searches) > 1 or prism_done
    if generate_comparisons:
        comparison_dir.mkdir(parents=True, exist_ok=True)
        print(f'\n  Generating comparison plots → {comparison_dir}')
        generate_strategy_comparison_plots(strategy_results, comparison_dir, skip=skip)


if __name__ == '__main__':
    main()
