#!/usr/bin/env python3

"""Regenerate all_experiments comparison plots from existing CSV results."""

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

sys.path.insert(0, str(REPO_ROOT))

from navigation_planta.reporting import (  # noqa: E402
    EXPERIMENT_COMPARISON_CONFIG,
    generate_strategy_comparison_plots,
)


EXCLUDED_RESULT_DIRS = {'camara_prism', 'comparison'}

STRATEGY_LABEL_BY_DIR = {
    'astar_blind___': 'astar(blind())',
    'eager_greedy__goalcount____': 'eager_greedy([goalcount()])',
    'lazy_greedy__ff____': 'lazy_greedy([ff()])',
}

EXPERIMENT_CSV_LOCATIONS = {
    'grid_map': Path('grid_map/adaptive/planning_times.csv'),
    'grid_map_no_sas': Path('grid_map/no-adaptation/planning_times.csv'),
    'camara_discretized': Path('camara_discretized/adaptive/planning_times.csv'),
    'camara_discretized_no_sas': Path('camara_discretized/no-adaptation/planning_times.csv'),
    'fd': Path('fd/adaptive/planning_times.csv'),
    'corridor': Path('corridor/adaptive/planning_times.csv'),
    'ma': Path('ma/adaptive/planning_times.csv'),
    'ma_no_sas': Path('ma/no-adaptation/planning_times.csv'),
    'combined': Path('combined/adaptive/planning_times.csv'),
    'combined_no_sas': Path('combined/no-adaptation/planning_times.csv'),
}


def _strategy_label(strategy_dir: Path) -> str:
    search_txt = strategy_dir / 'search.txt'
    if search_txt.is_file():
        label = search_txt.read_text(encoding='utf-8').strip()
        if label:
            return label

    metadata_json = strategy_dir / 'metadata.json'
    if metadata_json.is_file():
        try:
            metadata = json.loads(metadata_json.read_text(encoding='utf-8'))
        except json.JSONDecodeError:
            metadata = {}
        label = metadata.get('search')
        if isinstance(label, str) and label:
            return label

    return STRATEGY_LABEL_BY_DIR.get(strategy_dir.name, strategy_dir.name)


def discover_strategy_results(results_dir: Path) -> dict[str, dict[str, Path]]:
    """Discover comparison CSVs by probing the known all_experiments layout."""
    strategy_results: dict[str, dict[str, Path]] = {
        name: {} for name in EXPERIMENT_COMPARISON_CONFIG
    }

    for strategy_dir in sorted(results_dir.iterdir()):
        if not strategy_dir.is_dir() or strategy_dir.name in EXCLUDED_RESULT_DIRS:
            continue

        strategy_label = _strategy_label(strategy_dir)
        for exp_name, relative_csv in EXPERIMENT_CSV_LOCATIONS.items():
            csv_path = strategy_dir / relative_csv
            if csv_path.is_file():
                strategy_results[exp_name][strategy_label] = csv_path

    prism_csv = results_dir / 'camara_prism' / 'planning_times.csv'
    if prism_csv.is_file():
        strategy_results['camara_prism']['prism'] = prism_csv
        strategy_results['camara_discretized']['prism (baseline)'] = prism_csv
        strategy_results['camara_discretized_no_sas']['prism (baseline)'] = prism_csv

    return strategy_results


def _print_discovery_summary(strategy_results: dict[str, dict[str, Path]]) -> None:
    print('Discovered comparison CSVs:')
    for exp_name in sorted(strategy_results):
        csv_by_strategy = strategy_results[exp_name]
        if not csv_by_strategy:
            continue
        print(f'  {exp_name}:')
        for strategy_label, csv_path in csv_by_strategy.items():
            print(f'    {strategy_label}: {csv_path}')


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Regenerate comparison plots from an existing all_experiments result folder.')
    parser.add_argument(
        'results_dir',
        type=Path,
        help='Path to a results/all_experiments/<timestamp> folder.')
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help='Directory for regenerated plots (default: <results_dir>/comparison).')
    parser.add_argument(
        '--skip',
        nargs='*',
        default=[],
        metavar='NAME',
        help='Experiment names to skip.')
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print discovered CSVs without generating plots.')
    args = parser.parse_args()

    results_dir = args.results_dir.resolve()
    if not results_dir.is_dir():
        raise SystemExit(f'Results folder does not exist: {results_dir}')

    strategy_results = discover_strategy_results(results_dir)
    _print_discovery_summary(strategy_results)
    if args.dry_run:
        return

    comparison_dir = args.output_dir.resolve() if args.output_dir else results_dir / 'comparison'
    generated = generate_strategy_comparison_plots(
        strategy_results,
        comparison_dir,
        skip=set(args.skip),
    )
    print(f'Generated {len(generated)} comparison plots in {comparison_dir}')


if __name__ == '__main__':
    main()
