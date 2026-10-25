"""Source-generation contracts and real TeX/Poppler smoke test where available."""
from pathlib import Path
import shutil
import subprocess
from unittest.mock import patch

import pytest

from navigation_planta.paper_export import load_suite, write_data
from navigation_planta.paper_rendering import (
    BOX_STYLES, SUPPORT, box_plot, generate_sources, preflight, render_previews,
    run_command, sweep_ticks,
)
from test_paper_export import suite  # noqa: F401 -- shared synthetic suite fixture


def sources(tmp_path, suite, latex_prefix=''):
    source, profile = suite
    datasets, _, _ = load_suite(source, profile)
    root = tmp_path / 'sources'
    for folder in ('data', 'plots', 'tables', 'standalone', 'summaries'):
        (root / folder).mkdir(parents=True)
    endpoints, camara = write_data(root, datasets)
    fragments = generate_sources(root, datasets, endpoints, camara, latex_prefix=latex_prefix)
    return root, fragments


def test_generated_sources(suite, tmp_path):
    root, fragments = sources(tmp_path, suite)
    assert len(fragments) == 29
    for fragment in fragments:
        text = (root / fragment).read_text()
        assert str(suite[0]) not in text
        assert (root / 'standalone' / fragment.name).exists()
    main = (root / 'plots/camara_map_move_actions.tex').read_text()
    extra = (root / 'plots/camara_map_move_actions_astar.tex').read_text()
    assert 'camaraPddlMoveMED' in main
    assert 'camaraPddlAstarMoveMED' not in main
    assert 'camaraPddlAstarMoveMED' in extra
    assert 'comparable_move_outliers.csv' in main
    assert 'no_sas' in (root / 'plots/s1_time_plot.tex').read_text()
    assert 'no_sas' in (root / 'plots/s4_time_plot.tex').read_text()
    assert 'no_sas' not in (root / 'plots/s1_action_plot.tex').read_text()
    assert 'no_sas' not in (root / 'plots/s5_time_plot.tex').read_text()
    assert r'\label{tab:scalability_summary}' in (root / 'tables/scalability_summary.tex').read_text()


@pytest.mark.parametrize('sid', ['s1', 's2', 's3', 's4', 's5'])
def test_scalability_memory_plots(suite, tmp_path, sid):
    root, fragments = sources(tmp_path, suite, latex_prefix='nested/export')
    path = Path(f'plots/{sid}_memory_plot.tex')
    assert path in fragments
    text = (root / path).read_text()
    assert 'ylabel={RSS Peak Memory [MiB]}' in text
    assert 'y=mean_peak_memory,' in text
    assert 'y=mean_peak_memory_plus_std,' in text
    assert 'y=mean_peak_memory_minus_std,' in text
    assert r'\addlegendentry{A* mean}' in text
    assert r'\addlegendentry{EG mean}' in text
    assert text.count(r'\addlegendentry') == 2
    assert 'no_sas' not in text
    assert 'nested/export/data/' in text
    assert 'ymax=44' in text  # Synthetic data: 40 MiB, SD 0, plus 10% headroom.
    assert (root / 'standalone' / path.name).exists()


def test_scalability_table_original_style(suite, tmp_path):
    root, _ = sources(tmp_path, suite)
    text = (root / 'tables/scalability_summary.tex').read_text()
    assert r'\resizebox{.75\textwidth}{!}' in text
    assert r'\begin{tabular}{lcccccccc}' in text
    assert r'\multirow{3}{*}{\textbf{Exp.}}' in text
    assert r'\multicolumn{4}{c}{\textbf{A*}}' in text
    assert r'\multicolumn{4}{c}{\textbf{Eager greedy}}' in text
    assert text.count(r'\textbf{Planning time (s)}') == 2
    assert text.count(r'\textbf{Plan length}') == 2
    assert text.count(r'\textbf{Mean}') == 4
    assert text.count(r'\textbf{Std.}') == 4
    assert r'\cmidrule(lr){2-3}\cmidrule(lr){4-5}\cmidrule(lr){6-7}\cmidrule(lr){8-9}' in text
    for macro in ('Sone', 'Stwo', 'Sthree', 'Sfour', 'Sfive'):
        assert f'\\{macro} &' in text
        row = next(line for line in text.splitlines() if line.startswith(f'\\{macro} &'))
        values = row.removesuffix(r' \\').split(' & ')[1:]
        assert len(values) == 8
        assert all(value == f'{float(value):.2f}' for value in values)
        assert f'\\{macro}: 2 ' in text  # Caption endpoints come from the synthetic suite.
        assert f'\\providecommand{{\\{macro}}}' in (root / 'standalone/scalability_summary.tex').read_text()
    # The other table keeps its existing width and alignment.
    other = (root / 'tables/camara_summary.tex').read_text()
    assert r'\resizebox{\linewidth}{!}' in other
    assert r'\begin{tabular}{lrrrrrrr}' in other


def test_original_plot_styling(suite, tmp_path):
    root, fragments = sources(tmp_path, suite)
    for fragment in fragments:
        if fragment.parts[0] != 'plots':
            continue
        text = (root / fragment).read_text()
        assert r'baseline=(paperAxis.north)' in text
        assert r'name=paperAxis' in text
        assert r'width=\textwidth' in text
        assert r'height=0.75\textwidth' in text
        assert r'tick label style={font=\small}' not in text
        if fragment.name.startswith('camara_'):
            assert 'xmin=0,\nxmax=3' in text
            assert 'line width=0.75pt' in text
            assert 'opacity=0.2,line width=0.5pt' in text
            assert 'node[left,text=' in text
            assert 'boxplot box cs:' in text
            assert 'axis cs:' not in text
            assert 'fixed,precision=2' not in text
            assert r'font=\scriptsize' not in text
        else:
            assert r'font=\footnotesize,nodes={scale=0.8,transform shape}' in text
            assert 'legend image post style={scale=0.5}' in text
            assert '!20,fill opacity=0.3' in text
            assert 'fill opacity=0.16' not in text
            assert 'thick,color=' in text
    assert r'\providecommand{\paperEagerLabel}{\acronym}' in SUPPORT
    assert r'\providecommand{\paperAstarLabel}{\acronym{} (A*)}' in SUPPORT
    assert r'\textsc{Camara}' in SUPPORT
    assert '{cmyk}{0.64,0,0.95,0.40}' in SUPPORT
    assert r'\usepackage[tt=false]{libertine}' in (root / 'standalone/s1_time_plot.tex').read_text()
    time = (root / 'plots/s1_time_plot.tex').read_text()
    assert 'xtick={0,200,400,600,800,900}' in time
    assert r'\addlegendentry{No adap. mean}' in time


@pytest.mark.parametrize('metric', BOX_STYLES)
def test_box_axes_preserve_style_and_include_extrema(metric):
    style = BOX_STYLES[metric]
    def plot(low, high):
        stats = {'metrics': {metric: {'min': low, 'max': high}}}
        return box_plot('eager', metric, 'Test', 'unused', {'prism': stats, 'eager': stats})
    # Interior observations retain the original view and explicit ticks.
    low = style['min']
    high = style['max']
    text = plot(low, high)
    assert f'ymin={style["min"]:g},' in text
    assert f'ymax={style["max"]:g},' in text
    assert 'ytick={' + ','.join(f'{v:g}' for v in style['ticks']) + '}' in text
    # A changed experiment range must not be clipped to historical limits.
    text = plot(.1 if style['min'] else 0, style['max'] * 10)
    limits = dict(line.split('=', 1) for line in text.splitlines() if line.startswith(('ymin=', 'ymax=')))
    assert float(limits['ymin'].rstrip(',')) <= (.1 if style['min'] else 0)
    assert float(limits['ymax'].rstrip(',')) > style['max'] * 10


def test_sweep_tick_extension():
    assert sweep_ticks('s3', 2, 4) == (2, 4, [2, 3, 4])
    low, high, ticks = sweep_ticks('s5', 9, 1000)
    assert (low, high) == (9, 1000)
    assert ticks == [200, 400, 600, 800, 1000]


def test_missing_tool_and_subprocess_errors():
    with patch('navigation_planta.paper_rendering.shutil.which', return_value=None):
        with pytest.raises(RuntimeError, match='Missing rendering executables'):
            preflight()
    with patch('navigation_planta.paper_rendering.subprocess.run',
               return_value=subprocess.CompletedProcess(['pdflatex'], 1, stdout='TeX detail', stderr='failure')):
        with pytest.raises(RuntimeError, match='TeX detail'):
            run_command(['pdflatex'])
    with patch('navigation_planta.paper_rendering.subprocess.run',
               side_effect=subprocess.TimeoutExpired(['pdflatex'], 1)):
        with pytest.raises(RuntimeError, match='Could not execute'):
            run_command(['pdflatex'])


@pytest.mark.parametrize('latex_prefix', ['', 'nested/navigation_experiment'])
def test_real_tex_and_png_smoke(suite, tmp_path, latex_prefix):
    if not all(shutil.which(tool) for tool in ('pdflatex', 'pdftoppm', 'kpsewhich')):
        pytest.skip('Real preview smoke test requires TeX Live and Poppler')
    preflight()  # Existing but broken installations must fail, not be silently skipped.
    root, fragments = sources(tmp_path, suite, latex_prefix)
    render_previews(root, fragments, latex_prefix=latex_prefix)
    for fragment in fragments:
        assert (root / fragment.with_suffix('.pdf')).read_bytes().startswith(b'%PDF')
        assert (root / fragment.with_suffix('.png')).read_bytes().startswith(b'\x89PNG')
