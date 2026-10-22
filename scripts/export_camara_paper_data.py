#!/usr/bin/env python3
"""Export separate Cámara comparison datasets from a completed experiment suite.

Examples (run from navigation_planta):
    python scripts/export_camara_paper_data.py results/all_experiments/<timestamp> \
        --output-dir results/paper_export
    python scripts/export_camara_paper_data.py results/all_experiments/<timestamp> \
        --strategy astar --output-dir ../planta_paper_overleaf/data

Both strategies are exported by default, with _astar/_eager suffixes. Selecting
one strategy produces camara_map_pddl_experiment.csv, compatible with the
existing two-box paper figures. PRISM is always exported. This script does not
run experiments, modify LaTeX, or regenerate box statistics.
"""

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import sys

STRATEGIES = {
    'astar': 'astar(blind())',
    'eager': 'eager_greedy([goalcount()])',
}
COMMON_COLUMNS = (
    'init_goal', 'time', 'action_count', 'move_action', 'reconfig_action',
    'peak_memory', 'remaining_battery', 'distance',
)
MANIFEST_NAME = 'camara_export_manifest.json'


def validate_csv(content: bytes, source: Path, *, pddl: bool) -> Counter:
    """Validate measurements without changing or filtering source rows."""
    reader = csv.DictReader(io.StringIO(content.decode('utf-8-sig'), newline=''))
    required = (*COMMON_COLUMNS, *(('comparable_action_count',) if pddl else ()))
    headers = reader.fieldnames or []
    if len(headers) != len(set(headers)):
        raise ValueError(f'{source}: duplicate column names')
    missing = set(required) - set(headers)
    if missing:
        raise ValueError(f'{source}: missing columns: {sorted(missing)}')
    counts = Counter()
    for line, row in enumerate(reader, start=2):
        if None in row or any(value is None for value in row.values()):
            raise ValueError(f'{source}:{line}: malformed CSV row')
        pair = row['init_goal'].strip()
        if not pair:
            raise ValueError(f'{source}:{line}: empty init_goal')
        if 'mode' in row and row['mode'] != 'adaptive':
            raise ValueError(f'{source}:{line}: expected adaptive results')
        for column in required[1:]:
            try:
                value = float(row[column])
            except (ValueError, TypeError) as exc:
                raise ValueError(f'{source}:{line}: invalid {column}') from exc
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{source}:{line}: invalid {column}: {value}')
            if column.endswith('action') or column.endswith('count'):
                if not value.is_integer():
                    raise ValueError(f'{source}:{line}: non-integer {column}')
        if float(row['action_count']) == 0:
            raise ValueError(f'{source}:{line}: no-plan/zero-action observation')
        counts[pair] += 1
    if not counts:
        raise ValueError(f'{source}: empty dataset')
    if len(set(counts.values())) != 1:
        raise ValueError(f'{source}: inconsistent repetition counts across pairs')
    return counts


def export_datasets(suite_dir: Path, output_dir: Path, strategies: list[str],
                    *, overwrite: bool = False) -> list[Path]:
    suite_dir = suite_dir.resolve()
    output_dir = output_dir.resolve()
    if not strategies or len(strategies) != len(set(strategies)):
        raise ValueError('Select at least one strategy, without duplicates')
    sources = []
    for strategy in strategies:
        if strategy not in STRATEGIES:
            raise ValueError(f'Unknown strategy: {strategy}')
        search = STRATEGIES[strategy]
        matches = [p.parent for p in sorted(suite_dir.glob('*/search.txt'))
                   if p.read_text(encoding='utf-8').strip() == search]
        if len(matches) != 1:
            raise ValueError(f'Expected exactly one search.txt for {search}; '
                             f'found {len(matches)} in {suite_dir}')
        suffix = '' if len(strategies) == 1 else f'_{strategy}'
        sources.append((strategy, search,
                        matches[0] / 'camara_discretized/adaptive/planning_times.csv',
                        f'camara_map_pddl_experiment{suffix}.csv'))
    sources.append(('prism', None, suite_dir / 'camara_prism/planning_times.csv',
                    'camara_map_prism_experiment.csv'))

    # Read and validate everything before creating or replacing any output.
    payloads = {}
    entries = []
    reference_counts = None
    for label, search, source, filename in sources:
        content = source.read_bytes()
        counts = validate_csv(content, source, pddl=(label != 'prism'))
        if reference_counts is not None and counts != reference_counts:
            raise ValueError(f'{source}: initial-goal coverage or repetition '
                             'counts do not match the other datasets')
        reference_counts = counts
        destination = output_dir / filename
        if destination == source.resolve():
            raise ValueError(f'Refusing to overwrite source: {source}')
        payloads[destination] = content
        entries.append({
            'strategy': label, 'search': search,
            'mode': 'adaptive' if label != 'prism' else None,
            'source': str(source), 'destination': filename,
            'sha256': hashlib.sha256(content).hexdigest(),
            'rows': sum(counts.values()), 'initial_goal_pairs': len(counts),
            'repetitions_per_pair': next(iter(counts.values())),
        })
    manifest = {
        'schema_version': 1,
        'exported_at_utc': datetime.now(timezone.utc).isoformat(),
        'suite_directory': str(suite_dir),
        'pooled_strategies': False,
        'datasets': entries,
    }
    payloads[output_dir / MANIFEST_NAME] = (json.dumps(manifest, indent=2) + '\n').encode()
    for destination in payloads:
        if destination.exists() and not overwrite:
            raise FileExistsError(f'{destination} exists; use --overwrite to replace it')
        if destination.exists() and not destination.is_file():
            raise ValueError(f'Not a regular file: {destination}')
    output_dir.mkdir(parents=True, exist_ok=True)
    for destination, content in payloads.items():
        destination.write_bytes(content)
    return list(payloads)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('suite_dir', type=Path, help='Completed all_experiments timestamp directory')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--strategy', nargs='+', choices=STRATEGIES,
                        default=list(STRATEGIES),
                        help='Strategies to export (default: astar eager); never pooled')
    parser.add_argument('--overwrite', action='store_true',
                        help='Allow replacement of existing export files and manifest')
    args = parser.parse_args()
    try:
        paths = export_datasets(args.suite_dir, args.output_dir, args.strategy,
                                overwrite=args.overwrite)
    except (OSError, ValueError) as exc:
        print(f'Export failed: {exc}', file=sys.stderr)
        return 1
    for path in paths:
        print(f'Exported: {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
