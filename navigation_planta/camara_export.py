"""Shared source discovery and legacy Cámara CSV validation (no planners)."""

from collections import Counter
import csv
import io
import math
from pathlib import Path

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



def discover_strategies(suite_dir: Path) -> dict[str, Path]:
    directories = {}
    for strategy, search in STRATEGIES.items():
        matches = [p.parent for p in sorted(suite_dir.glob("*/search.txt"))
                   if p.read_text(encoding="utf-8").strip() == search]
        if len(matches) != 1:
            raise ValueError(f"Expected exactly one search.txt for {search}; "
                             f"found {len(matches)} in {suite_dir}")
        directories[strategy] = matches[0]
    return directories
