"""Validated, transactional export of completed navigation experiments.

This module deliberately does not import experiment runners or invoke planners.
"""

from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
import shutil
import sys
import tempfile

from camara_export import COMMON_COLUMNS, STRATEGIES, discover_strategies, validate_csv
from paper_statistics import box_statistics, comparable_moves, sweep_statistics

# Stable manifest identifier: preserve compatibility with existing export bundles.
EXPORTER = 'navigation_planta.paper_export'
SCHEMA_VERSION = 1
MANUSCRIPT_ROOT = Path(__file__).resolve().parents[2] / 'planta_paper_overleaf'


def validate_latex_prefix(prefix):
    """Restrict generated TeX paths to safe, relative POSIX components."""
    if prefix and (any(part in ('', '.', '..') for part in prefix.split('/')) or
                   not re.fullmatch(r'[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*', prefix)):
        raise ValueError('latex-prefix must be a safe relative POSIX path (no spaces or ..)')
    return prefix


@dataclass(frozen=True)
class PaperProfile:
    """Explicit completeness contract, injectable for small synthetic tests."""
    name: str = 'navigation-paper-v1'
    repetitions: int = 10
    levels: dict = field(default_factory=lambda: {
        'grid_map': tuple(range(9, 901, 9)), 'fd': (2, 4, 6, 8, 10, 12, 15),
        'corridor': (2, 3, 4), 'ma': (1, 2, 3, 4, 5),
        'combined': tuple(range(9, 91, 9)),
    })


# id, experiment folder, grouping column, time column, x label
SWEEPS = (
    ('s1', 'grid_map', 'nodes', 'time', 'Number of nodes'),
    ('s2', 'fd', 'n_fd', 'planning_time', 'Architectural configurations'),
    ('s3', 'corridor', 'k', 'planning_time', 'Corridor types'),
    ('s4', 'ma', 'm', 'planning_time', 'Mission-level actions'),
    ('s5', 'combined', 'nodes', 'planning_time', 'Number of nodes'),
)
SWEEP_PLOT_METRICS = ('time', 'action', 'memory')
# metric, macro suffix, plot suffix, axis label
CAMARA_METRICS = (
    ('time', 'Time', 'time', 'Planning time (seconds)'),
    ('peak_memory', 'Mem', 'memory', 'Estimated peak aggregate RSS (MiB)'),
    ('remaining_battery', 'Battery', 'battery', 'Remaining battery'),
    ('distance', 'Distance', 'distance', 'Distance traveled (m)'),
    ('move_action', 'Move', 'move_actions', 'Number of move actions'),
    ('reconfig_action', 'Reconfig', 'reconfig_actions', 'Number of reconfiguration actions'),
)
METHOD_LABELS = {'prism': 'Cámara et al. (replicated)',
                 'eager': 'PLANTA (eager greedy)', 'astar': 'PLANTA (A*)'}
METHOD_STEMS = {'eager': 'camara_map_pddl_experiment',
                'astar': 'camara_map_pddl_experiment_astar',
                'prism': 'camara_map_prism_experiment'}
METHOD_PREFIXES = {'eager': 'camaraPddl', 'astar': 'camaraPddlAstar', 'prism': 'camaraPrism'}


def sweep_stem(sid, strategy, mode='adaptive'):
    search = 'a_star' if sid == 's1' and strategy == 'astar' else strategy
    no_sas = '_no_sas' if mode == 'no-adaptation' else ''
    return f'{sid}_{search}{no_sas}_experiment'


def csv_bytes(content, source, required, mode):
    """Parse strictly, converting only required numeric measurements."""
    reader = csv.DictReader(io.StringIO(content.decode('utf-8-sig'), newline=''), strict=True)
    headers = reader.fieldnames or []
    if len(headers) != len(set(headers)):
        raise ValueError(f'{source}: duplicate column names')
    missing = set(required) - set(headers)
    if missing:
        raise ValueError(f'{source}: missing columns: {sorted(missing)}')
    def records():
        try:
            yield from reader
        except csv.Error as exc:
            raise ValueError(f'{source}:{reader.line_num}: malformed CSV: {exc}') from exc

    rows = []
    for line, raw in enumerate(records(), start=2):
        context = f'{source}:{line}'
        if None in raw or any(value is None for value in raw.values()):
            raise ValueError(f'{context}: malformed CSV row')
        if 'mode' in raw and raw['mode'] != mode:
            raise ValueError(f'{context}: expected mode {mode}')
        row = dict(raw)
        row['_source_row'] = line
        for name in required:
            if name == 'init_goal':
                continue
            try:
                value = float(raw[name])
            except (ValueError, TypeError) as exc:
                raise ValueError(f'{context}: invalid {name}') from exc
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{context}: invalid {name}: {value}')
            if (name.endswith('action') or name.endswith('count')) and not value.is_integer():
                raise ValueError(f'{context}: non-integer {name}')
            row[name] = value
        if row['action_count'] == 0:
            raise ValueError(f'{context}: no-plan/zero-action observation')
        accounted = row['move_action'] + row['reconfig_action']
        if (row['action_count'] < accounted or
                ('init_goal' in required and row['action_count'] != accounted)):
            raise ValueError(f'{context}: inconsistent move/reconfiguration/total action counts')
        if 'comparable_action_count' in required and not (
                row['reconfig_action'] <= row['comparable_action_count'] <= row['action_count']):
            raise ValueError(f'{context}: invalid comparable_action_count')
        rows.append(row)
    if not rows:
        raise ValueError(f'{source}: empty dataset')
    return rows


def load_suite(suite, profile=PaperProfile()):
    """Read all inputs once; returned bytes are also the bytes later exported."""
    if profile.repetitions < 2:
        raise ValueError('Profile needs at least two repetitions for sample SD')
    directories = discover_strategies(suite)
    datasets, absent = [], []
    for sid, experiment, group, time, xlabel in SWEEPS:
        for strategy, directory in directories.items():
            modes = ['adaptive'] + (['no-adaptation'] if sid in ('s1', 's4', 's5') else [])
            for mode in modes:
                source = directory / experiment / mode / 'planning_times.csv'
                if not source.exists() and mode == 'no-adaptation' and strategy == 'eager':
                    absent.append(str(source))
                    continue
                content = source.read_bytes()
                required = [group, time, 'action_count', 'move_action', 'reconfig_action', 'peak_memory']
                rows = csv_bytes(content, source, required, mode)
                counts = Counter(row[group] for row in rows)
                expected = {x: profile.repetitions for x in profile.levels[experiment]}
                if counts != expected:
                    raise ValueError(f'{source}: incomplete parameter grid/repetitions; '
                                     f'expected {expected}, got {dict(counts)}')
                processed, summary = sweep_statistics(rows, group, time)
                datasets.append(dict(kind='sweep', sid=sid, experiment=experiment,
                                     strategy=strategy, mode=mode, group=group, time=time,
                                     xlabel=xlabel, source=source, content=content, rows=rows,
                                     stem=sweep_stem(sid, strategy, mode),
                                     processed=processed, summary=summary))
    reference = None
    waypoint_set = None
    for method in ('eager', 'astar', 'prism'):
        source = (suite / 'camara_prism/planning_times.csv' if method == 'prism' else
                  directories[method] / 'camara_discretized/adaptive/planning_times.csv')
        content = source.read_bytes()
        counts = validate_csv(content, source, pddl=method != 'prism')
        required = list(COMMON_COLUMNS) + ([] if method == 'prism' else ['comparable_action_count'])
        rows = csv_bytes(content, source, required, 'adaptive')
        pairs = set()
        for pair in counts:
            match = re.fullmatch(r'(wp\d+)_(wp\d+)', pair)
            if not match or match[1] == match[2]:
                raise ValueError(f'{source}: invalid initial-goal pair: {pair}')
            pairs.add((match[1], match[2]))
        waypoints = {wp for pair in pairs for wp in pair}
        complete = {(start, goal) for start in waypoints for goal in waypoints if start != goal}
        if pairs != complete or set(counts.values()) != {profile.repetitions}:
            raise ValueError(f'{source}: incomplete directed initial-goal coverage/repetitions')
        if reference is not None and counts != reference:
            raise ValueError(f'{source}: initial-goal coverage/repetitions do not match other methods')
        reference, waypoint_set = counts, sorted(waypoints)
        metrics = {}
        for metric, _, _, _ in CAMARA_METRICS:
            values = [comparable_moves(row, method) if metric == 'move_action' else row[metric]
                      for row in rows]
            metrics[metric] = box_statistics(values)
        datasets.append(dict(kind='camara', strategy=method, mode='adaptive', source=source,
                             content=content, rows=rows, stem=METHOD_STEMS[method], metrics=metrics))
    return datasets, absent, waypoint_set


def write_csv(path, rows, headers=None):
    rows = list(rows)
    if not rows and headers is None:
        raise ValueError(f'{path}: no CSV schema')
    with path.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=headers or list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n',
                    encoding='utf-8')


def write_data(root, datasets):
    """Write data and full-precision summaries; return structured table inputs."""
    groups, camara_rows, endpoints = [], [], []
    camara = {}
    for dataset in datasets:
        stem = dataset['stem']
        (root / 'data' / f'{stem}.csv').write_bytes(dataset['content'])
        if dataset['kind'] == 'sweep':
            write_csv(root / 'data' / f'{stem}_processed.csv', dataset['processed'])
            for summary in dataset['summary']:
                for metric, stats in summary['metrics'].items():
                    groups.append(dict(experiment=dataset['sid'], strategy=dataset['strategy'],
                                       mode=dataset['mode'], x=summary['x'], metric=metric, **stats))
            endpoint = dataset['processed'][-1]
            endpoints.append(dict(experiment=dataset['sid'], strategy=dataset['strategy'],
                                  mode=dataset['mode'], **endpoint))
            continue
        method = dataset['strategy']
        camara[method] = dataset
        for metric, suffix, _, _ in CAMARA_METRICS:
            filename_metric = 'comparable_move' if metric == 'move_action' and method != 'prism' else metric
            filename = f'{stem}_{filename_metric}'
            stats = dataset['metrics'][metric]
            prefix = METHOD_PREFIXES[method] + suffix
            macros = [('LW', 'lower_whisker'), ('LQ', 'q1'), ('MED', 'median'),
                      ('UQ', 'q3'), ('UW', 'upper_whisker')]
            text = (f'% Generated by {EXPORTER}; n={stats["n"]}; Tukey 1.5 IQR; '
                    f'{stats["outlier_count"]} outliers, {len(stats["outlier_marks"])} rank-sampled marks.\n')
            text += ''.join(f'\\def\\{prefix}{macro}{{{stats[key]:.6f}}}\n' for macro, key in macros)
            (root / 'data' / f'{filename}_boxstats.tex').write_text(text, encoding='utf-8')
            write_csv(root / 'data' / f'{filename}_outliers.csv',
                      [{'y': f'{value:.6f}'} for value in stats['outlier_marks']], ['y'])
            camara_rows.append(dict(method=method, metric=metric,
                                    **{key: value for key, value in stats.items() if key != 'outlier_marks'}))
    write_csv(root / 'summaries/scalability_groups.csv', groups)
    write_json(root / 'summaries/scalability_groups.json', groups)
    write_json(root / 'summaries/scalability_endpoints.json', endpoints)
    # Normalize heterogeneous grouping-column names to x for the endpoint CSV.
    endpoint_csv = []
    for row in endpoints:
        record = dict(row)
        group = next(name for name in ('nodes', 'n_fd', 'k', 'm') if name in record)
        x = record.pop(group)
        endpoint_csv.append(dict(experiment=record.pop('experiment'), strategy=record.pop('strategy'),
                                 mode=record.pop('mode'), x=x, **record))
    write_csv(root / 'summaries/scalability_endpoints.csv', endpoint_csv)
    write_csv(root / 'summaries/camara_metrics.csv', camara_rows)
    write_json(root / 'summaries/camara_metrics.json', camara_rows)
    details, comparisons, comparison_rows, reconfiguration_rows = {}, {}, [], []
    for method, dataset in camara.items():
        rows = dataset['rows']
        maximum = max(row['time'] for row in rows)
        frequencies = Counter(int(row['reconfig_action']) for row in rows)
        details[method] = {
            'worst_time_observations': [dict(source_row=row['_source_row'], init_goal=row['init_goal'],
                                             time=row['time'], peak_memory=row['peak_memory'])
                                        for row in rows if row['time'] == maximum],
            'reconfiguration_distribution': dict(sorted(frequencies.items())),
            'fraction_above_one_reconfiguration': sum(n for k, n in frequencies.items() if k > 1) / len(rows),
        }
        for count, n in sorted(frequencies.items()):
            reconfiguration_rows.append(dict(method=method, reconfigurations=count, n=n, fraction=n / len(rows)))
    for method in ('eager', 'astar'):
        comparisons[method] = {}
        for metric in ('time', 'peak_memory'):
            numerator = camara['prism']['metrics'][metric]['median']
            denominator = camara[method]['metrics'][metric]['median']
            if denominator <= 0:
                raise ValueError(f'{method}: nonpositive median {metric}; ratio undefined')
            value = numerator / denominator
            comparisons[method][f'prism_over_pddl_median_{metric}'] = value
            comparison_rows.append(dict(strategy=method, comparison=f'prism_over_pddl_median_{metric}',
                                        numerator=numerator, denominator=denominator, value=value))
        prism_battery = camara['prism']['metrics']['remaining_battery']['median']
        pddl_battery = camara[method]['metrics']['remaining_battery']['median']
        if prism_battery <= 0:
            raise ValueError('PRISM median battery must be positive for percentage comparison')
        percent = 100 * (pddl_battery - prism_battery) / prism_battery
        comparisons[method]['pddl_minus_prism_battery_percent_of_prism'] = percent
        comparison_rows.append(dict(strategy=method, comparison='pddl_minus_prism_battery_percent_of_prism',
                                    numerator=pddl_battery - prism_battery,
                                    denominator=prism_battery, value=percent))
    write_json(root / 'summaries/camara_details.json', details)
    write_csv(root / 'summaries/camara_worst_observations.csv',
              [dict(method=method, **row) for method, item in details.items()
               for row in item['worst_time_observations']])
    write_csv(root / 'summaries/camara_reconfigurations.csv', reconfiguration_rows)
    write_json(root / 'summaries/camara_comparisons.json', comparisons)
    write_csv(root / 'summaries/camara_comparisons.csv', comparison_rows)
    baseline_comparisons = []
    for sid, _, _, _, _ in SWEEPS:
        for strategy in STRATEGIES:
            selected = [d for d in datasets if d['kind'] == 'sweep' and
                        d['sid'] == sid and d['strategy'] == strategy]
            by_mode = {d['mode']: d for d in selected}
            if 'no-adaptation' not in by_mode:
                continue
            for adaptive, baseline in zip(by_mode['adaptive']['processed'], by_mode['no-adaptation']['processed']):
                denominator = baseline['mean_time']
                if denominator <= 0:
                    raise ValueError(f'{sid}/{strategy}: baseline mean time is nonpositive')
                baseline_comparisons.append(dict(experiment=sid, strategy=strategy,
                    x=adaptive[by_mode['adaptive']['group']], adaptive_mean_time=adaptive['mean_time'],
                    no_adaptation_mean_time=denominator,
                    adaptive_over_no_adaptation=adaptive['mean_time'] / denominator))
    write_csv(root / 'summaries/no_adaptation_comparisons.csv', baseline_comparisons)
    write_json(root / 'summaries/no_adaptation_comparisons.json', baseline_comparisons)
    return endpoints, camara


def artifact_inventory(datasets, *, previews=True):
    """List the complete public output contract before any files are generated."""
    paths = {'manifest.json', 'README.md', 'plots/paper_support.tex'}
    for dataset in datasets:
        stem = dataset['stem']
        paths.add(f'data/{stem}.csv')
        if dataset['kind'] == 'sweep':
            paths.add(f'data/{stem}_processed.csv')
        else:
            for metric, _, _, _ in CAMARA_METRICS:
                name = 'comparable_move' if metric == 'move_action' and dataset['strategy'] != 'prism' else metric
                paths.update((f'data/{stem}_{name}_boxstats.tex', f'data/{stem}_{name}_outliers.csv'))
    names = [f'plots/{sid}_{metric}_plot' for sid, *_ in SWEEPS for metric in SWEEP_PLOT_METRICS]
    names += [f'plots/camara_map_{suffix}{extra}' for _, _, suffix, _ in CAMARA_METRICS
              for extra in ('', '_astar')]
    names += ['tables/scalability_summary', 'tables/camara_summary']
    for name in names:
        paths.update(name + extension for extension in (('.tex', '.pdf', '.png') if previews else ('.tex',)))
        if previews:
            paths.add(f'standalone/{Path(name).name}.tex')
    paths.update(('tables/scalability_summary.csv', 'tables/camara_summary.csv'))
    for name in ('scalability_groups', 'scalability_endpoints', 'camara_metrics',
                 'camara_comparisons', 'no_adaptation_comparisons'):
        paths.update(f'summaries/{name}.{extension}' for extension in ('csv', 'json'))
    paths.update(('summaries/camara_details.json', 'summaries/camara_worst_observations.csv',
                  'summaries/camara_reconfigurations.csv'))
    return sorted(paths)


def check_output(suite, output, overwrite):
    """Resolve paths only after refusing symlink components and dangerous roots."""
    absolute = output.absolute()
    if any(part.is_symlink() for part in (absolute, *absolute.parents)):
        raise ValueError(f'Refusing symlink output path: {output}')
    output = absolute.resolve()
    if output == suite or suite in output.parents or output in suite.parents:
        raise ValueError('Output and source suite must not overlap')
    paper = MANUSCRIPT_ROOT.resolve()
    if (output == paper or output in paper.parents or
            (paper in output.parents and output != paper / 'navigation_experiment')):
        raise ValueError('Only the dedicated navigation_experiment bundle may be inside the manuscript checkout')
    if output.is_dir() and not any(output.iterdir()):
        return output  # Allow an explicitly created, empty destination.
    if output.exists():
        if not overwrite:
            raise FileExistsError(f'{output} exists; use --overwrite for an exporter-owned bundle')
        marker = output / 'manifest.json'
        try:
            manifest = json.loads(marker.read_text(encoding='utf-8'))
        except (OSError, ValueError) as exc:
            raise ValueError(f'Refusing replacement of non-exporter directory: {output}') from exc
        if manifest.get('exporter') != EXPORTER or manifest.get('schema_version') != SCHEMA_VERSION:
            raise ValueError(f'Refusing replacement of non-exporter directory: {output}')
        # Refuse unknown/user-added files and symlinks before eventual cleanup.
        allowed = set(manifest.get('outputs', {})) | {'manifest.json'}
        allowed_directories = {parent.as_posix() for name in allowed
                               for parent in Path(name).parents if parent != Path('.')}
        actual = set()
        for path in output.rglob('*'):
            if path.is_symlink():
                raise ValueError(f'Refusing symlink inside existing bundle: {path}')
            relative = path.relative_to(output).as_posix()
            if path.is_dir() and relative not in allowed_directories:
                raise ValueError(f'Refusing replacement with untracked directory: {path}')
            if path.is_file():
                actual.add(relative)
                if relative not in allowed:
                    raise ValueError(f'Refusing replacement with untracked file: {path}')
                if relative != 'manifest.json' and hashlib.sha256(path.read_bytes()).hexdigest() != manifest['outputs'][relative]:
                    raise ValueError(f'Refusing replacement of modified artifact: {path}')
        if actual != allowed:
            raise ValueError('Existing bundle is incomplete; refusing replacement')
    return output


def generate_paper_artifacts(suite, output, *, overwrite=False, dry_run=False,
                             profile=PaperProfile(), no_previews=False, latex_prefix=''):
    """Validate, stage, render, then publish. Return the manifest/discovery report."""
    from paper_rendering import generate_sources, preflight, render_previews

    latex_prefix = validate_latex_prefix(latex_prefix)
    suite = Path(suite).resolve(strict=True)
    output = check_output(suite, Path(output), overwrite)
    datasets, absent, waypoints = load_suite(suite, profile)
    versions = {'python': sys.version.split()[0]} if no_previews else preflight()
    report = dict(exporter=EXPORTER, schema_version=SCHEMA_VERSION, profile=profile.name,
                  suite_directory=str(suite), output_directory=str(output),
                  exported_at_utc=datetime.now(timezone.utc).isoformat(),
                  main_camara_strategy='eager', pooled_strategies=False,
                  statistics=dict(std_ddof=1, quartiles='linear/type-7', whisker_iqr=1.5,
                                  max_outlier_marks=250, memory_unit='MiB',
                                  timing='planner only; excludes OWL preprocessing',
                                  camara_variability='across all routes and repetitions; not timing noise alone'),
                  versions=versions, waypoints=waypoints, optional_absent=absent,
                  expected_repetitions=profile.repetitions, expected_levels=profile.levels,
                  implementation_sha256={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                         for name in ('paper_export.py', 'paper_statistics.py',
                                                      'paper_rendering.py', 'camara_export.py')},
                  latex_prefix=latex_prefix, previews_enabled=not no_previews,
                  planned_previews=dict(plots=0 if no_previews else
                                       len(SWEEPS) * len(SWEEP_PLOT_METRICS) + 2 * len(CAMARA_METRICS),
                                       tables=0 if no_previews else 2),
                  planned_artifacts=artifact_inventory(datasets, previews=not no_previews),
                  datasets=[dict(source=str(d['source']), destination=f'data/{d["stem"]}.csv',
                                 sha256=hashlib.sha256(d['content']).hexdigest(), rows=len(d['rows']),
                                 strategy=d['strategy'], search=STRATEGIES.get(d['strategy']),
                                 mode=d['mode'],
                                 groups=(len(d['processed']) if d['kind'] == 'sweep' else len(waypoints) * (len(waypoints) - 1)),
                                 repetitions=profile.repetitions) for d in datasets])
    if dry_run:
        return report
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f'.{output.name}.staging-', dir=output.parent))
    backup = None
    try:
        for folder in ('data', 'plots', 'tables', 'summaries') + (() if no_previews else ('standalone',)):
            (stage / folder).mkdir()
        endpoints, camara = write_data(stage, datasets)
        fragments = generate_sources(stage, datasets, endpoints, camara,
                                     latex_prefix=latex_prefix, previews=not no_previews)
        if not no_previews:
            render_previews(stage, fragments, latex_prefix=latex_prefix)
        readme = BUNDLE_README.replace('@PREFIX@', latex_prefix + '/' if latex_prefix else '')
        readme += ('\nPreview rendering and standalone wrappers were disabled for this export.\n'
                   if no_previews else '\nPDF/PNG previews and standalone wrappers are included.\n')
        (stage / 'README.md').write_text(readme, encoding='utf-8')
        report['outputs'] = {p.relative_to(stage).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in sorted(stage.rglob('*')) if p.is_file()}
        if set(report['outputs']) | {'manifest.json'} != set(report['planned_artifacts']):
            raise RuntimeError('Generated artifact inventory does not match the export contract')
        write_json(stage / 'manifest.json', report)
        # Recheck ownership/integrity after rendering before touching an older bundle.
        check_output(suite, output, overwrite)
        if output.exists():
            backup = Path(tempfile.mkdtemp(prefix=f'.{output.name}.backup-', dir=output.parent))
            backup.rmdir()
            output.rename(backup)
        try:
            stage.rename(output)
        except OSError:
            if backup is not None:
                backup.rename(output)
                backup = None
            raise
        if backup is not None:
            shutil.rmtree(backup)
        return report
    finally:
        if stage.exists():
            shutil.rmtree(stage)


BUNDLE_README = """# Generated navigation paper artifacts

Main Cámara comparison: adaptive eager greedy versus PRISM. Additional `_astar`
figures use adaptive A*; strategies are never pooled. No experiments were rerun.

`data/` preserves raw bytes and the paper's legacy filenames and processed columns.
Scalability processed CSVs also include mean_peak_memory, std_peak_memory, and
mean_peak_memory_plus_std/minus_std in MiB. Each S1--S5 memory plot compares
adaptive A* and eager greedy, using the mean with a sample-standard-deviation band.
`plots/*.tex` are includable PGFPlots fragments. From a LaTeX project root containing
these directories, load TikZ, PGFPlots (compat=1.18), statistics and fillbetween
libraries, and `\\input{@PREFIX@plots/paper_support.tex}` in the preamble. Then use
`\\input{@PREFIX@plots/s1_time_plot.tex}` etc. The support file uses fallback label/color
macros, retaining definitions from a manuscript preamble. Tables need booktabs,
multirow, and graphicx. The scalability table preserves `tab:scalability_summary`;
replace, rather than duplicate, any existing table with that label.

When enabled, standalone wrappers in `standalone/` compile from the LaTeX project
root, with this bundle located at the configured prefix. PDF previews are compiled
from the same fragments; PNG previews are those PDFs at 300 DPI. `--no-previews`
omits wrappers and rendered files and needs no TeX/Poppler tools.

Statistics: mean +/- sample SD (ddof=1), not standard error/confidence intervals.
Quartiles use linear/type-7 interpolation; Tukey whiskers use 1.5 IQR. At most
250 rank-sampled outlier marks are drawn, preserving multiplicities. All statistics
use all observations. Full-precision summaries are in `summaries/`; displayed TeX
box statistics use six decimal places for legacy compatibility.

PDDL move counts are comparable_action_count minus reconfig_action, not the number
of discretized movements. PRISM uses move_action. Memory is estimated peak aggregate
RSS in MiB. Planning times exclude OWL preprocessing and are not end-to-end times.
Cámara SD pools route differences and runtime variability. Matching route keys does
not imply identical objectives or safety formulations; scalability rows are not
claimed to be paired instances. Ratios are PRISM median / PDDL median. Battery
percent differences are 100 * (PDDL median - PRISM median) / PRISM median. All ties
for worst timing are retained with source row numbers in the summaries.

No PDDL complexity tables or manuscript prose are generated. Integration into the
paper and updates to prose, captions, and descriptions remain manual next steps.
`manifest.json` records input/output hashes, statistical settings, and tool versions.
"""
