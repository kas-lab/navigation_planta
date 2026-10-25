"""Synthetic suites test export contracts without invoking experiments."""
import csv
import hashlib
import json
from pathlib import Path
from unittest.mock import patch

import pytest

from navigation_planta.camara_export import STRATEGIES
from navigation_planta.paper_export import (
    PaperProfile, SWEEPS, check_output, generate_paper_artifacts, load_suite, write_csv,
)


@pytest.fixture
def suite(tmp_path):
    root = tmp_path / 'suite'
    profile = PaperProfile(name='synthetic', repetitions=2,
                           levels={exp: (1, 2) for _, exp, _, _, _ in SWEEPS})
    for strategy, search in STRATEGIES.items():
        folder = root / strategy
        folder.mkdir(parents=True)
        (folder / 'search.txt').write_text(search)
        for sid, exp, group, time, _ in SWEEPS:
            for mode in ('adaptive', 'no-adaptation') if sid in ('s1', 's4', 's5') else ('adaptive',):
                path = folder / exp / mode / 'planning_times.csv'
                path.parent.mkdir(parents=True)
                rows = [dict([(group, x), (time, t + (2 if strategy == 'astar' else 0)),
                              ('action_count', 6), ('move_action', 3), ('reconfig_action', 1),
                              ('peak_memory', 40), ('remaining_battery', '-1'), ('mode', mode)])
                        for x in (1, 2) for t in (1, 2)]
                write_csv(path, rows)
        path = folder / 'camara_discretized/adaptive/planning_times.csv'
        path.parent.mkdir(parents=True)
        rows = [dict(init_goal=pair, time=t + (2 if strategy == 'astar' else 0),
                     action_count=5, move_action=3, reconfig_action=2, peak_memory=40,
                     comparable_action_count=3, remaining_battery=100, distance=3)
                for pair in ('wp1_wp2', 'wp2_wp1') for t in (1, 2)]
        write_csv(path, rows)
    path = root / 'camara_prism/planning_times.csv'
    path.parent.mkdir()
    rows = [dict(init_goal=pair, time=t, action_count=3, move_action=2, reconfig_action=1,
                 peak_memory=100, remaining_battery=110, distance=3)
            for pair in ('wp1_wp2', 'wp2_wp1') for t in (10, 11)]
    write_csv(path, rows)
    return root, profile


def snapshot(root):
    return {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.rglob('*') if p.is_file()}


def fake_render(root, fragments, **kwargs):
    for path in fragments:
        for suffix in ('.pdf', '.png'):
            (root / path.with_suffix(suffix)).write_bytes(b'fake preview')


@pytest.fixture
def mocked_tools():
    with patch('navigation_planta.paper_rendering.preflight', return_value={'test': 'mock'}), \
            patch('navigation_planta.paper_rendering.render_previews', side_effect=fake_render):
        yield


def test_bundle_contract_main_eager_separate_astar(suite, tmp_path, mocked_tools):
    root, profile = suite
    original = snapshot(root)
    output = tmp_path / 'export'
    report = generate_paper_artifacts(root, output, profile=profile)
    assert snapshot(root) == original
    assert report['main_camara_strategy'] == 'eager'
    assert len(list((output / 'plots').glob('*.tex'))) == 28  # 27 figures + shared support
    assert len(list((output / 'plots').glob('*.pdf'))) == 27
    assert len(list((output / 'tables').glob('*.csv'))) == 2
    assert (output / 'data/camara_map_pddl_experiment.csv').read_bytes() == (
        root / 'eager/camara_discretized/adaptive/planning_times.csv').read_bytes()
    assert (output / 'data/camara_map_pddl_experiment_astar.csv').read_bytes() == (
        root / 'astar/camara_discretized/adaptive/planning_times.csv').read_bytes()
    assert r'\def\camaraPddlTimeMED{1.500000}' in (
        output / 'data/camara_map_pddl_experiment_time_boxstats.tex').read_text()
    assert r'\def\camaraPddlAstarTimeMED{3.500000}' in (
        output / 'data/camara_map_pddl_experiment_astar_time_boxstats.tex').read_text()
    assert r'\def\camaraPddlMoveMED{1.000000}' in (
        output / 'data/camara_map_pddl_experiment_comparable_move_boxstats.tex').read_text()
    assert report['statistics']['std_ddof'] == 1
    assert report['expected_repetitions'] == 2
    assert report['datasets'][0]['search'] == STRATEGIES['astar']
    assert set(report['planned_artifacts']) == set(snapshot(output))
    assert report['outputs'] == {k: v for k, v in snapshot(output).items() if k != 'manifest.json'}
    table = list(csv.DictReader((output / 'tables/scalability_summary.csv').open()))
    processed = list(csv.DictReader((output / 'data/s1_a_star_experiment_processed.csv').open()))
    assert table[0]['astar_mean_time'] == processed[-1]['mean_time']
    assert float(processed[0]['mean_peak_memory']) == 40
    assert float(processed[0]['std_peak_memory']) == 0
    for sid, *_ in SWEEPS:
        assert f'plots/{sid}_memory_plot.tex' in report['outputs']
        assert f'plots/{sid}_memory_plot.pdf' in report['outputs']
        assert f'plots/{sid}_memory_plot.png' in report['outputs']
    with pytest.raises(FileExistsError):
        generate_paper_artifacts(root, output, profile=profile)
    before = snapshot(output)
    generate_paper_artifacts(root, output, profile=profile, overwrite=True)
    after = snapshot(output)
    assert {k: v for k, v in before.items() if k != 'manifest.json'} == {
        k: v for k, v in after.items() if k != 'manifest.json'}


def test_dry_run_writes_nothing(suite, tmp_path, mocked_tools):
    root, profile = suite
    out = tmp_path / 'new-parent/export'
    report = generate_paper_artifacts(root, out, profile=profile, dry_run=True)
    assert report['planned_previews'] == dict(plots=27, tables=2)
    assert not out.parent.exists()


@pytest.mark.parametrize('bad', ['missing', 'nan', 'negative', 'fraction', 'duplicate',
                                 'truncated', 'inconsistent', 'mode', 'malformed', 'quoting'])
def test_bad_sweeps_fail_without_output(suite, tmp_path, mocked_tools, bad):
    root, profile = suite
    path = root / 'astar/grid_map/adaptive/planning_times.csv'
    text = path.read_text()
    if bad == 'missing':
        path.unlink()
    else:
        lines = text.splitlines()
        if bad == 'nan':
            lines[1] = lines[1].replace(',3,6,', ',nan,6,')
        elif bad == 'negative':
            lines[1] = lines[1].replace(',3,6,', ',-1,6,')
        elif bad == 'fraction':
            lines[1] = lines[1].replace(',6,3,', ',6.5,3,')
        elif bad == 'duplicate':
            lines[0] += ',time'
        elif bad == 'truncated':
            lines.pop()
        elif bad == 'inconsistent':
            lines[1] = lines[1].replace(',6,3,', ',1,3,')
        elif bad == 'mode':
            lines[1] = lines[1].replace('adaptive', 'no-adaptation')
        elif bad == 'malformed':
            lines[1] += ',extra'
        elif bad == 'quoting':
            lines[1] += ',"unterminated'
        path.write_text('\n'.join(lines) + '\n')
    out = tmp_path / 'export'
    with pytest.raises((ValueError, FileNotFoundError)):
        generate_paper_artifacts(root, out, profile=profile)
    assert not out.exists()


def test_missing_or_duplicate_strategy(suite):
    root, profile = suite
    (root / 'astar/search.txt').unlink()
    with pytest.raises(ValueError, match='found 0'):
        load_suite(root, profile)
    (root / 'astar/search.txt').write_text(STRATEGIES['astar'])
    extra = root / 'extra'
    extra.mkdir()
    (extra / 'search.txt').write_text(STRATEGIES['astar'])
    with pytest.raises(ValueError, match='found 2'):
        load_suite(root, profile)


def test_optional_baselines_and_unset_battery(suite):
    root, profile = suite
    (root / 'eager/grid_map/no-adaptation/planning_times.csv').unlink()
    data, absent, _ = load_suite(root, profile)
    assert len(absent) == 1
    assert len(data) == 18
    assert data[0]['rows'][0]['remaining_battery'] == '-1'


@pytest.mark.parametrize('bad', ['missing_pair_all', 'repetition', 'derived', 'count', 'missing_column'])
def test_camara_validation(suite, bad):
    root, profile = suite
    paths = [root / f'{s}/camara_discretized/adaptive/planning_times.csv' for s in STRATEGIES]
    paths.append(root / 'camara_prism/planning_times.csv')
    for path in paths if bad == 'missing_pair_all' else paths[:1]:
        lines = path.read_text().splitlines()
        if bad == 'missing_pair_all':
            lines = [line for line in lines if not line.startswith('wp2_wp1')]
        elif bad == 'repetition':
            lines.pop()
        elif bad == 'derived':
            lines[1] = lines[1].replace(',3,100,3', ',1,100,3')
        elif bad == 'count':
            lines[1] = lines[1].replace(',5,3,2,', ',4,3,2,')
        elif bad == 'missing_column':
            lines[0] = lines[0].replace('distance', 'other')
        path.write_text('\n'.join(lines) + '\n')
    with pytest.raises(ValueError):
        load_suite(root, profile)


def test_failure_preserves_existing_bundle(suite, tmp_path, mocked_tools):
    root, profile = suite
    out = tmp_path / 'export'
    generate_paper_artifacts(root, out, profile=profile)
    original, inputs = snapshot(out), snapshot(root)
    with patch('navigation_planta.paper_rendering.render_previews', side_effect=RuntimeError('TeX failed')):
        with pytest.raises(RuntimeError, match='TeX failed'):
            generate_paper_artifacts(root, out, profile=profile, overwrite=True)
    assert snapshot(out) == original
    assert snapshot(root) == inputs
    assert not list(tmp_path.glob('.export.staging-*'))


def test_output_safety(suite, tmp_path, mocked_tools):
    root, profile = suite
    for path in (root, root / 'export', root.parent):
        with pytest.raises(ValueError, match='overlap'):
            check_output(root, path, False)
    link = tmp_path / 'link'
    link.symlink_to(root, target_is_directory=True)
    with pytest.raises(ValueError, match='symlink'):
        check_output(root, link / 'export', False)
    arbitrary = tmp_path / 'arbitrary'
    arbitrary.mkdir()
    (arbitrary / 'keep').write_text('user data')
    with pytest.raises(ValueError, match='non-exporter'):
        check_output(root, arbitrary, True)
    out = tmp_path / 'export'
    generate_paper_artifacts(root, out, profile=profile)
    (out / 'user.txt').write_text('preserve me')
    with pytest.raises(ValueError, match='untracked'):
        check_output(root, out, True)
    (out / 'user.txt').unlink()
    (out / 'empty-user-dir').mkdir()
    with pytest.raises(ValueError, match='untracked directory'):
        check_output(root, out, True)
    (out / 'empty-user-dir').rmdir()
    (out / 'user-link').symlink_to(root, target_is_directory=True)
    with pytest.raises(ValueError, match='symlink'):
        check_output(root, out, True)
    (out / 'user-link').unlink()
    (out / 'README.md').write_text('user changes')
    with pytest.raises(ValueError, match='modified'):
        check_output(root, out, True)


def test_publication_failure_rolls_back(suite, tmp_path, mocked_tools):
    root, profile = suite
    out = tmp_path / 'export'
    generate_paper_artifacts(root, out, profile=profile)
    original = snapshot(out)
    rename = Path.rename

    def fail_stage(path, target):
        if '.staging-' in path.name:
            raise OSError('simulated publication error')
        return rename(path, target)

    with patch.object(Path, 'rename', fail_stage):
        with pytest.raises(OSError, match='publication'):
            generate_paper_artifacts(root, out, profile=profile, overwrite=True)
    assert snapshot(out) == original


def test_no_previews_needs_no_rendering_tools(suite, tmp_path):
    root, profile = suite
    out = tmp_path / 'export'
    out.mkdir()  # An existing empty destination is safe without --overwrite.
    with patch('navigation_planta.paper_rendering.preflight', side_effect=AssertionError('tool check')), \
            patch('navigation_planta.paper_rendering.render_previews', side_effect=AssertionError('render')):
        report = generate_paper_artifacts(root, out, profile=profile, no_previews=True,
                                          latex_prefix='navigation_experiment')
    assert not (out / 'standalone').exists()
    assert not list(out.rglob('*.pdf'))
    assert not list(out.rglob('*.png'))
    assert (out / 'plots/paper_support.tex').exists()
    assert report['planned_previews'] == dict(plots=0, tables=0)
    assert report['previews_enabled'] is False
    assert report['latex_prefix'] == 'navigation_experiment'
    assert set(report['planned_artifacts']) == set(snapshot(out))
    text = (out / 'plots/camara_map_time.tex').read_text()
    assert r'\input{navigation_experiment/data/' in text
    assert '{navigation_experiment/data/s1_' in (out / 'plots/s1_time_plot.tex').read_text()
    assert r'\input{navigation_experiment/plots/paper_support.tex}' in (out / 'README.md').read_text()
    generate_paper_artifacts(root, out, profile=profile, no_previews=True,
                             latex_prefix='navigation_experiment', overwrite=True)


@pytest.mark.parametrize('prefix', ['/absolute', '../escape', 'a/../b', 'bad path', 'a//b',
                                    'a/', 'a%comment', r'a\b', '.'])
def test_invalid_latex_prefix(suite, tmp_path, prefix):
    root, profile = suite
    with pytest.raises(ValueError, match='latex-prefix'):
        generate_paper_artifacts(root, tmp_path / 'out', profile=profile,
                                 latex_prefix=prefix, no_previews=True)
    assert not (tmp_path / 'out').exists()


def test_only_dedicated_manuscript_bundle_is_allowed(suite, tmp_path):
    root, profile = suite
    paper = tmp_path / 'paper'
    paper.mkdir()
    (paper / 'main.tex').write_text('preserve manuscript')
    with patch('navigation_planta.paper_export.MANUSCRIPT_ROOT', paper):
        for forbidden in (paper, paper / 'data', paper / 'navigation_experiment/nested'):
            with pytest.raises(ValueError, match='dedicated'):
                check_output(root, forbidden, False)
        target = paper / 'navigation_experiment'
        target.mkdir()
        generate_paper_artifacts(root, target, profile=profile, no_previews=True,
                                 latex_prefix='navigation_experiment')
        assert (paper / 'main.tex').read_text() == 'preserve manuscript'
        (target / 'unmanaged.tex').write_text('user content')
        with pytest.raises(ValueError, match='untracked'):
            generate_paper_artifacts(root, target, profile=profile, no_previews=True, overwrite=True)
