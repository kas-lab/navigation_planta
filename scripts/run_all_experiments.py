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

import matplotlib.pyplot as plt
import numpy as np

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

# Per-experiment CSV schema for comparison plots
EXPERIMENT_CSV_CONFIG = {
    'grid_map':                  {'x': 'nodes',     'y': 'time',          'xlabel': 'Number of Nodes',   'numeric': True},
    'grid_map_no_sas':           {'x': 'nodes',     'y': 'time',          'xlabel': 'Number of Nodes',   'numeric': True},
    # 'camara_pddl':             {'x': 'init_goal', 'y': 'time',          'xlabel': 'Init→Goal Pair',    'numeric': False},
    'camara_discretized':        {'x': 'init_goal', 'y': 'time',          'xlabel': 'Init→Goal Pair',    'numeric': False, 'box': True},
    'camara_discretized_no_sas': {'x': 'init_goal', 'y': 'time',          'xlabel': 'Init→Goal Pair',    'numeric': False, 'box': True},
    'camara_prism':              {'x': 'init_goal', 'y': 'time',          'xlabel': 'Init→Goal Pair',    'numeric': False},
    'fd':                        {'x': 'n_fd',      'y': 'planning_time', 'xlabel': 'N FunctionDesigns', 'numeric': True},
    'corridor':                  {'x': 'k',         'y': 'planning_time', 'xlabel': 'K Corridor Types',  'numeric': True},
    'ma':                        {'x': 'm',         'y': 'planning_time', 'xlabel': 'M Mission Actions',  'numeric': True},
    'ma_no_sas':                 {'x': 'm',         'y': 'planning_time', 'xlabel': 'M Mission Actions',  'numeric': True},
    'combined':                  {'x': 'nodes',     'y': 'planning_time', 'xlabel': 'Number of Nodes',   'numeric': True},
    'combined_no_sas':           {'x': 'nodes',     'y': 'planning_time', 'xlabel': 'Number of Nodes',   'numeric': True},
}

PDDL_EXPERIMENT_NAMES = set(EXPERIMENT_CSV_CONFIG.keys())

# Display labels and order for camara box plots.
# Keys are the raw strategy identifiers used in strategy_csvs.
CAMARA_BOX_LABEL_MAP: dict[str, str] = {
    'prism (baseline)':           'Cámara et al. (2020)',
    'eager_greedy([goalcount()])': 'PDDL-SAS-first',
    'astar(blind())':              'PDDL-SAS-best',
}
CAMARA_BOX_ORDER: list[str] = [
    'prism (baseline)',
    'eager_greedy([goalcount()])',
    'astar(blind())',
]


def _sanitize(s: str) -> str:
    return re.sub(r'[^a-zA-Z0-9_-]', '_', s)[:60]


def plot_strategy_comparison(
        exp_name: str,
        strategy_csvs: dict[str, Path],
        out_dir: Path,
        y_col: str | None = None,
        ylabel: str = 'Mean Planning Time (seconds)',
        plot_suffix: str = '') -> None:
    """Plot mean y_col vs independent variable for all strategies.

    When y_col is None the default y column from EXPERIMENT_CSV_CONFIG is used.
    """
    config = EXPERIMENT_CSV_CONFIG.get(exp_name)
    if config is None or len(strategy_csvs) < 2:
        return

    x_col: str = config['x']
    resolved_y: str = y_col if y_col is not None else config['y']
    x_label: str = config['xlabel']
    x_numeric: bool = config['numeric']
    use_box: bool = config.get('box', False)

    fig, ax = plt.subplots(figsize=(10, 5))
    colors = plt.get_cmap('tab10').colors  # type: ignore[attr-defined]

    if use_box:
        box_data: list[np.ndarray] = []
        box_labels: list[str] = []
        box_colors: list = []
        is_camara = exp_name.startswith('camara_discretized')
        ordered_items = (
            sorted(strategy_csvs.items(),
                   key=lambda kv: CAMARA_BOX_ORDER.index(kv[0])
                   if kv[0] in CAMARA_BOX_ORDER else len(CAMARA_BOX_ORDER))
            if is_camara else list(strategy_csvs.items())
        )
        for i, (strat_label, csv_path) in enumerate(ordered_items):
            try:
                data = np.genfromtxt(
                    csv_path, delimiter=',', names=True, dtype=None, encoding='utf-8')
            except Exception as exc:
                print(f'  Warning: could not read {csv_path}: {exc}')
                continue
            if data.ndim == 0:
                data = data.reshape(1)
            if resolved_y not in data.dtype.names:
                continue
            ys = data[resolved_y].astype(float)
            if y_col == 'action_count':
                ys = ys[ys != -1]
            else:
                ys = ys[~np.isnan(ys)]
            display_label = CAMARA_BOX_LABEL_MAP.get(strat_label, strat_label) if is_camara else strat_label
            box_data.append(ys)
            box_labels.append(display_label)
            box_colors.append(colors[i % len(colors)])

        if box_data:
            bp = ax.boxplot(
                box_data,
                patch_artist=True,
                showmeans=True,
                labels=box_labels,
                meanprops=dict(marker='o', markerfacecolor='red', markeredgecolor='black'),
                medianprops=dict(color='orange'),
                whiskerprops=dict(color='black'),
            )
            for patch, color in zip(bp['boxes'], box_colors):
                patch.set_facecolor(color)
                patch.set_alpha(0.7)
            ax.set_xticklabels(box_labels, rotation=15, ha='right')
    else:
        for i, (strat_label, csv_path) in enumerate(strategy_csvs.items()):
            try:
                data = np.genfromtxt(
                    csv_path, delimiter=',', names=True, dtype=None, encoding='utf-8')
            except Exception as exc:
                print(f'  Warning: could not read {csv_path}: {exc}')
                continue
            if data.ndim == 0:
                data = data.reshape(1)
            if resolved_y not in data.dtype.names:
                continue

            color = colors[i % len(colors)]

            if x_numeric:
                xs = data[x_col].astype(float)
                ys = data[resolved_y].astype(float)
                if y_col == 'action_count':
                    ys = np.where(ys == -1, float('nan'), ys)
                unique_x = np.unique(xs)
                mean_y = np.array([np.nanmean(ys[xs == x]) for x in unique_x])
                std_y = np.array([np.nanstd(ys[xs == x]) for x in unique_x])
                ax.plot(unique_x, mean_y, marker='o', markersize=4,
                        label=strat_label, color=color)
                ax.fill_between(unique_x, mean_y - std_y, mean_y + std_y,
                                alpha=0.15, color=color)
            else:
                xs = data[x_col].astype(str)
                ys = data[resolved_y].astype(float)
                if y_col == 'action_count':
                    ys = np.where(ys == -1, float('nan'), ys)
                unique_x = list(dict.fromkeys(xs))
                n = len(strategy_csvs)
                width = 0.8 / n
                x_pos = np.arange(len(unique_x)) + i * width
                mean_y = [float(np.nanmean(ys[xs == x])) if (xs == x).any() else 0.0
                          for x in unique_x]
                ax.bar(x_pos, mean_y, width=width, label=strat_label,
                       color=color, alpha=0.8)
                ax.set_xticks(np.arange(len(unique_x)) + 0.4 - width / 2)
                ax.set_xticklabels(unique_x, rotation=45, ha='right')

        ax.legend(fontsize=8)

    ax.set_xlabel(x_label)
    ax.set_ylabel(ylabel)
    ax.set_title(f'Search Strategy Comparison — {exp_name}')
    ax.set_ylim(bottom=0)
    ax.grid(True)

    plot_path = out_dir / f'comparison_{exp_name}{plot_suffix}.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'  Comparison plot saved to {plot_path}')


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

        print(f'\n{"#" * 60}')
        print(f'  Strategy: {search_str}')
        print(f'{"#" * 60}')

        csv = _run_one('grid_map', 'Grid map scalability (vary nodes, PDDL)',
                       run_grid_map, strat_dir / 'grid_map',
                       mode='adaptive', runs=runs, show_plot=False, search=search_str)
        if csv:
            strategy_results['grid_map'][strat_label] = csv

        csv = _run_one('grid_map_no_sas', 'Grid map scalability no adaptation (vary nodes, PDDL)',
                       run_grid_map, strat_dir / 'grid_map',
                       mode='no-adaptation', runs=runs, show_plot=False, search=search_str)
        if csv:
            strategy_results['grid_map_no_sas'][strat_label] = csv

        # csv = _run_one('camara_pddl', 'Camara 2020 map — PDDL (PLANTA)',
        #                run_camara, strat_dir / 'camara_pddl',
        #                mode='adaptive', runs=runs, search=search_str)
        # if csv:
        #     strategy_results['camara_pddl'][strat_label] = csv

        csv = _run_one('camara_discretized', 'Camara 2020 map — discretized PDDL',
                       run_camara_discretized, strat_dir / 'camara_discretized',
                       mode='adaptive', runs=runs, search=search_str)
        if csv:
            strategy_results['camara_discretized'][strat_label] = csv

        csv = _run_one('camara_discretized_no_sas', 'Camara 2020 map — discretized PDDL - no adaptation',
                       run_camara_discretized, strat_dir / 'camara_discretized',
                       mode='no-adaptation', runs=runs, search=search_str)
        if csv:
            strategy_results['camara_discretized_no_sas'][strat_label] = csv

        csv = _run_one('fd', 'Exp A — FD scaling (vary FunctionDesigns)',
                       run_fd, strat_dir / 'fd',
                       n_runs=runs, mode='adaptive', search=search_str)
        if csv:
            strategy_results['fd'][strat_label] = csv

        csv = _run_one('corridor', 'Exp B — Corridor type scaling',
                       run_corridor, strat_dir / 'corridor',
                       n_runs=runs, mode='adaptive', search=search_str)
        if csv:
            strategy_results['corridor'][strat_label] = csv

        csv = _run_one('ma', 'Exp C — Mission action scaling',
                       run_ma, strat_dir / 'ma',
                       n_runs=runs, mode='adaptive', search=search_str)
        if csv:
            strategy_results['ma'][strat_label] = csv

        csv = _run_one('ma_no_sas', 'Exp C — Mission action scaling - no adaptation',
                       run_ma, strat_dir / 'ma',
                       n_runs=runs, mode='no-adaptation', search=search_str)
        if csv:
            strategy_results['ma_no_sas'][strat_label] = csv

        csv = _run_one('combined', 'Exp D — Combined scalability',
                       run_combined, strat_dir / 'combined',
                       n_runs=runs, mode='adaptive', search=search_str)
        if csv:
            strategy_results['combined'][strat_label] = csv

        csv = _run_one('combined_no_sas', 'Exp D — Combined scalability - no-adaptation',
                       run_combined, strat_dir / 'combined',
                       n_runs=runs, mode='no-adaptation', search=search_str)
        if csv:
            strategy_results['combined_no_sas'][strat_label] = csv

    # PRISM baseline has no --search; run it only once across all strategies
    if not prism_done:
        prism_csv = _run_one(
            'camara_prism', 'Camara 2020 map — PRISM (baseline)',
            run_prism, base_out / 'camara_prism')
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
        for exp_name, csv_by_strategy in strategy_results.items():
            if exp_name in skip:
                continue
            # camara_prism is a single series by design; all others need >= 2
            min_series = 1 if exp_name == 'camara_prism' else 2
            if len(csv_by_strategy) >= min_series:
                plot_strategy_comparison(exp_name, csv_by_strategy, comparison_dir)
                plot_strategy_comparison(
                    exp_name, csv_by_strategy, comparison_dir,
                    y_col='action_count',
                    ylabel='Mean Plan Length (actions)',
                    plot_suffix='_actions')
                if exp_name.startswith('camara_discretized'):
                    plot_strategy_comparison(
                        exp_name, csv_by_strategy, comparison_dir,
                        y_col='peak_memory',
                        ylabel='Peak Memory (MB)',
                        plot_suffix='_memory')


if __name__ == '__main__':
    main()
