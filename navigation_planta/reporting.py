"""Shared plotting and summary helpers for navigation experiments."""

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


NO_PLAN = -1


@dataclass(frozen=True)
class StrategyLineComparisonConfig:
    x_column: str
    y_column: str
    xlabel: str


@dataclass(frozen=True)
class StrategyBoxComparisonConfig:
    y_column: str
    xlabel: str


LINE_EXPERIMENT_COMPARISON_CONFIG = {
    'grid_map': StrategyLineComparisonConfig(
        x_column='nodes',
        y_column='time',
        xlabel='Number of Nodes',
    ),
    'grid_map_no_sas': StrategyLineComparisonConfig(
        x_column='nodes',
        y_column='time',
        xlabel='Number of Nodes',
    ),
    'fd': StrategyLineComparisonConfig(
        x_column='n_fd',
        y_column='planning_time',
        xlabel='N FunctionDesigns',
    ),
    'corridor': StrategyLineComparisonConfig(
        x_column='k',
        y_column='planning_time',
        xlabel='K Corridor Types',
    ),
    'ma': StrategyLineComparisonConfig(
        x_column='m',
        y_column='planning_time',
        xlabel='M Mission Actions',
    ),
    'ma_no_sas': StrategyLineComparisonConfig(
        x_column='m',
        y_column='planning_time',
        xlabel='M Mission Actions',
    ),
    'combined': StrategyLineComparisonConfig(
        x_column='nodes',
        y_column='planning_time',
        xlabel='Number of Nodes',
    ),
    'combined_no_sas': StrategyLineComparisonConfig(
        x_column='nodes',
        y_column='planning_time',
        xlabel='Number of Nodes',
    ),
}

BOX_EXPERIMENT_COMPARISON_CONFIG = {
    'camara_discretized': StrategyBoxComparisonConfig(
        y_column='time',
        xlabel='Method',
    ),
    'camara_discretized_no_sas': StrategyBoxComparisonConfig(
        y_column='time',
        xlabel='Method',
    ),
}

EXPERIMENT_COMPARISON_CONFIG = {
    **LINE_EXPERIMENT_COMPARISON_CONFIG,
    **BOX_EXPERIMENT_COMPARISON_CONFIG,
    'camara_prism': StrategyBoxComparisonConfig(
        y_column='time',
        xlabel='Method',
    ),
}

CAMARA_BOX_LABEL_MAP: dict[str, str] = {
    'prism (baseline)': 'Cámara et al. (2020)',
    'eager_greedy([goalcount()])': 'PDDL-SAS-first',
    'astar(blind())': 'PDDL-SAS-best',
}

CAMARA_BOX_ORDER: list[str] = [
    'prism (baseline)',
    'eager_greedy([goalcount()])',
    'astar(blind())',
]


def _record_mode(record) -> str:
    return record.mode


def _record_x_value(record):
    return record.x_value


def _record_planning_time(record) -> float:
    return record.planning_time


def _record_action_count(record) -> int:
    return record.action_count


def _record_peak_memory(record) -> float:
    return record.peak_memory


def _series_stats(values: Iterable[float]) -> tuple[float, float]:
    array = np.asarray(list(values), dtype=float)
    if array.size == 0:
        return float('nan'), 0.0
    return float(np.nanmean(array)), float(np.nanstd(array))


def summarize_mode_records(records: Sequence, mode_labels: dict[str, str]) -> list[str]:
    modes = list(mode_labels.keys())
    if len(modes) == 1:
        mode = modes[0]
        filtered = [record for record in records if _record_mode(record) == mode]
        times = [_record_planning_time(record) for record in filtered]
        counts = [
            _record_action_count(record)
            for record in filtered
            if _record_action_count(record) != NO_PLAN
        ]
        mems = [_record_peak_memory(record) for record in filtered]
        mean_t, std_t = _series_stats(times)
        ac_str = f' mean actions: {np.mean(counts):.1f}' if counts else ''
        max_mem = max(mems) if mems else 0.0
        return [f'Mean {mean_t:.4f}s Std dev: {std_t:.4f}s{ac_str} Max mem: {max_mem:.1f}MB']

    lines: list[str] = []
    for mode in modes:
        filtered = [record for record in records if _record_mode(record) == mode]
        times = [_record_planning_time(record) for record in filtered]
        counts = [
            _record_action_count(record)
            for record in filtered
            if _record_action_count(record) != NO_PLAN
        ]
        mems = [_record_peak_memory(record) for record in filtered]
        mean_t, std_t = _series_stats(times)
        ac_str = f' mean actions: {np.mean(counts):.1f}' if counts else ''
        max_mem = max(mems) if mems else 0.0
        lines.append(
            f'{mode_labels[mode]} mean {mean_t:.4f}s std dev: {std_t:.4f}s{ac_str} '
            f'max mem: {max_mem:.1f}MB'
        )
    return lines


def write_summary_report(folder: Path, lines: Sequence[str], filename: str = 'summary.txt') -> Path:
    report_path = folder / filename
    report_path.write_text('\n'.join(lines) + '\n')
    print(f'Summary saved to {report_path}')
    return report_path


def plot_numeric_sweep_results(
        folder: Path,
        records: Sequence,
        mode_labels: dict[str, str],
        *,
        xlabel: str,
        time_title: str,
        action_title: str,
        filename: str,
        x_values: Sequence | None = None,
        xticks: Sequence | None = None,
        time_ylabel: str = 'Mean Planning Time (seconds)',
        action_ylabel: str = 'Mean Plan Length (actions)',
        figsize: tuple[float, float] = (14, 5),
        markersize: float | None = None,
        show_plot: bool = False) -> Path:
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize)

    modes = list(mode_labels.keys())
    if x_values is None:
        x_values = sorted({_record_x_value(record) for record in records})

    plot_kwargs = {'marker': 'o', 'linestyle': '-'}
    if markersize is not None:
        plot_kwargs['markersize'] = markersize

    for mode in modes:
        mean_t, std_t, mean_ac, std_ac = [], [], [], []
        for x_value in x_values:
            mode_records = [
                record for record in records
                if _record_mode(record) == mode and _record_x_value(record) == x_value
            ]
            times = [_record_planning_time(record) for record in mode_records]
            mean_time, std_time = _series_stats(times)
            counts = [
                _record_action_count(record)
                for record in mode_records
                if _record_action_count(record) != NO_PLAN
            ]
            mean_count, std_count = _series_stats(counts)

            mean_t.append(mean_time)
            std_t.append(std_time)
            mean_ac.append(mean_count)
            std_ac.append(std_count)

        x_array = np.asarray(list(x_values), dtype=float)
        mean_t_array = np.asarray(mean_t, dtype=float)
        std_t_array = np.asarray(std_t, dtype=float)
        mean_ac_array = np.asarray(mean_ac, dtype=float)
        std_ac_array = np.asarray(std_ac, dtype=float)

        ax1.plot(x_array, mean_t_array, label=mode_labels[mode], **plot_kwargs)
        ax1.fill_between(x_array, mean_t_array - std_t_array, mean_t_array + std_t_array, alpha=0.2)

        valid_counts = ~np.isnan(mean_ac_array)
        if valid_counts.any():
            ax2.plot(
                x_array[valid_counts],
                mean_ac_array[valid_counts],
                label=mode_labels[mode],
                **plot_kwargs,
            )
            ax2.fill_between(
                x_array[valid_counts],
                mean_ac_array[valid_counts] - std_ac_array[valid_counts],
                mean_ac_array[valid_counts] + std_ac_array[valid_counts],
                alpha=0.2,
            )

    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(time_ylabel)
    ax1.set_ylim(bottom=0)
    ax1.set_title(time_title)
    ax1.legend()
    ax1.grid(True)

    ax2.set_xlabel(xlabel)
    ax2.set_ylabel(action_ylabel)
    ax2.set_ylim(bottom=0)
    ax2.set_title(action_title)
    handles, labels = ax2.get_legend_handles_labels()
    if handles:
        ax2.legend()
    ax2.grid(True)

    if xticks is not None:
        ax1.set_xticks(xticks)
        ax2.set_xticks(xticks)

    plot_path = folder / filename
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    if show_plot:
        plt.show(block=True)
    plt.close(fig)
    print(f'Plot saved to {plot_path}')
    return plot_path


def plot_memory_summary(
        folder: Path,
        records: Sequence,
        mode_labels: dict,
        filename: str = 'peak_memory_boxplot.png') -> Path:
    import matplotlib.pyplot as plt

    modes = list(mode_labels.keys())
    labels = []
    mean_mems, std_mems = [], []
    for mode in modes:
        mems = np.array([
            _record_peak_memory(record)
            for record in records
            if _record_mode(record) == mode
        ], dtype=float)
        if mems.size:
            labels.append(mode_labels[mode])
            mean_mems.append(mems.mean())
            std_mems.append(mems.std())
    if not mean_mems:
        return folder / filename

    fig, ax = plt.subplots(figsize=(max(6, 3 * len(mean_mems)), 5))
    x = np.arange(len(mean_mems))
    ax.bar(
        x,
        mean_mems,
        yerr=std_mems,
        capsize=5,
        color='mediumseagreen',
        alpha=0.8,
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('Peak Memory (MB)')
    ax.set_title('Peak Memory by Mode')
    ax.set_ylim(bottom=0)
    ax.grid(True, axis='y')

    plot_path = folder / filename
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')
    return plot_path


def plot_mode_summary(
        folder: Path,
        records: Sequence,
        mode_labels: dict,
        filename: str = 'camara_summary.png') -> Path:
    import matplotlib.pyplot as plt

    modes = list(mode_labels.keys())
    labels = [mode_labels[m] for m in modes]
    mean_times, std_times = [], []
    mean_actions, std_actions = [], []
    mean_mems, std_mems = [], []

    for mode in modes:
        times = np.array([
            _record_planning_time(record)
            for record in records
            if _record_mode(record) == mode
        ])
        counts = np.array(
            [
                _record_action_count(record)
                for record in records
                if _record_mode(record) == mode and _record_action_count(record) != NO_PLAN
            ],
            dtype=float,
        )
        mems = np.array([
            _record_peak_memory(record)
            for record in records
            if _record_mode(record) == mode
        ])
        mean_times.append(times.mean())
        std_times.append(times.std())
        mean_actions.append(counts.mean() if counts.size else float('nan'))
        std_actions.append(counts.std() if counts.size else 0.0)
        mean_mems.append(mems.mean())
        std_mems.append(mems.std())

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))
    x = np.arange(len(modes))

    ax1.bar(x, mean_times, yerr=std_times, capsize=5, color='steelblue', alpha=0.8)
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels)
    ax1.set_ylabel('Mean Planning Time (seconds)')
    ax1.set_title('Planning Time by Mode')
    ax1.set_ylim(bottom=0)
    ax1.grid(True, axis='y')

    ax2.bar(x, mean_actions, yerr=std_actions, capsize=5, color='darkorange', alpha=0.8)
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels)
    ax2.set_ylabel('Mean Plan Length (actions)')
    ax2.set_title('Plan Length by Mode')
    ax2.set_ylim(bottom=0)
    ax2.grid(True, axis='y')

    ax3.bar(x, mean_mems, yerr=std_mems, capsize=5, color='mediumseagreen', alpha=0.8)
    ax3.set_xticks(x)
    ax3.set_xticklabels(labels)
    ax3.set_ylabel('Mean Peak Memory (MB)')
    ax3.set_title('Peak Memory by Mode')
    ax3.set_ylim(bottom=0)
    ax3.grid(True, axis='y')

    plot_path = folder / filename
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')
    return plot_path


def _load_csv_data(csv_path: Path):
    data = np.genfromtxt(
        csv_path, delimiter=',', names=True, dtype=None, encoding='utf-8')
    if data.ndim == 0:
        data = data.reshape(1)
    return data


def _apply_plot_style(plt, style_name: str) -> None:
    if style_name in plt.style.available:
        plt.style.use(style_name)


def _strategy_metric_column(
        data,
        strategy_label: str,
        requested_y: str,
        *,
        drop_no_plan: bool) -> np.ndarray | None:
    metric_column = requested_y
    if requested_y == 'comparable_action_count' and strategy_label == 'prism (baseline)':
        metric_column = 'action_count'
    if metric_column not in data.dtype.names:
        return None

    values = data[metric_column].astype(float)
    if drop_no_plan:
        values = values[values != NO_PLAN]
        values = values[~np.isnan(values)]
    return values


def plot_strategy_line_comparison(
        exp_name: str,
        strategy_csvs: dict[str, Path],
        out_dir: Path,
        *,
        y_col: str | None = None,
        ylabel: str = 'Mean Planning Time (seconds)',
        plot_suffix: str = '') -> Path | None:
    import matplotlib.pyplot as plt

    config = LINE_EXPERIMENT_COMPARISON_CONFIG.get(exp_name)
    if config is None or len(strategy_csvs) < 2:
        return None

    resolved_y = y_col if y_col is not None else config.y_column
    fig, ax = plt.subplots(figsize=(10, 5))
    colors = plt.get_cmap('tab10').colors  # type: ignore[attr-defined]

    for index, (strategy_label, csv_path) in enumerate(strategy_csvs.items()):
        try:
            data = _load_csv_data(csv_path)
        except Exception as exc:
            print(f'  Warning: could not read {csv_path}: {exc}')
            continue
        ys = _strategy_metric_column(
            data,
            strategy_label,
            resolved_y,
            drop_no_plan=False,
        )
        if ys is None:
            continue
        if resolved_y == 'action_count':
            ys = np.where(ys == NO_PLAN, float('nan'), ys)

        xs = data[config.x_column].astype(float)
        color = colors[index % len(colors)]
        unique_x = np.unique(xs)
        mean_y = np.array([np.nanmean(ys[xs == x]) for x in unique_x])
        std_y = np.array([np.nanstd(ys[xs == x]) for x in unique_x])
        ax.plot(unique_x, mean_y, marker='o', markersize=4, label=strategy_label, color=color)
        ax.fill_between(unique_x, mean_y - std_y, mean_y + std_y, alpha=0.15, color=color)

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(fontsize=8)

    ax.set_xlabel(config.xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(f'Search Strategy Comparison — {exp_name}')
    ax.set_ylim(bottom=0)
    ax.grid(True)

    plot_path = out_dir / f'comparison_{exp_name}{plot_suffix}.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'  Comparison plot saved to {plot_path}')
    return plot_path


def plot_strategy_box_comparison(
        exp_name: str,
        strategy_csvs: dict[str, Path],
        out_dir: Path,
        *,
        y_col: str | None = None,
        ylabel: str = 'Planning Time (seconds)',
        plot_suffix: str = '') -> Path | None:
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    config = BOX_EXPERIMENT_COMPARISON_CONFIG.get(exp_name)
    if config is None or len(strategy_csvs) < 2:
        return None

    resolved_y = y_col if y_col is not None else config.y_column
    _apply_plot_style(plt, 'seaborn-v0_8-whitegrid')
    colors = plt.rcParams['axes.prop_cycle'].by_key().get('color')
    if not colors:
        colors = plt.get_cmap('tab10').colors  # type: ignore[attr-defined]

    ordered_items = sorted(
        strategy_csvs.items(),
        key=lambda kv: CAMARA_BOX_ORDER.index(kv[0])
        if kv[0] in CAMARA_BOX_ORDER else len(CAMARA_BOX_ORDER),
    )

    box_data: list[np.ndarray] = []
    box_labels: list[str] = []
    box_colors: list = []
    for index, (strategy_label, csv_path) in enumerate(ordered_items):
        try:
            data = _load_csv_data(csv_path)
        except Exception as exc:
            print(f'  Warning: could not read {csv_path}: {exc}')
            continue

        ys = _strategy_metric_column(
            data,
            strategy_label,
            resolved_y,
            drop_no_plan=(resolved_y in {'action_count', 'comparable_action_count'}),
        )
        if ys is None or ys.size == 0:
            continue

        box_data.append(ys)
        box_labels.append(CAMARA_BOX_LABEL_MAP.get(strategy_label, strategy_label))
        box_colors.append(colors[index % len(colors)])

    if not box_data:
        return None

    fig, ax = plt.subplots(figsize=(10, 5))
    box_plot = ax.boxplot(
        box_data,
        patch_artist=True,
        showmeans=True,
        labels=box_labels,
        meanprops=dict(marker='o', markerfacecolor='red', markeredgecolor='black'),
        medianprops=dict(color='black'),
        whiskerprops=dict(linestyle='--', color='black'),
    )
    for patch, color in zip(box_plot['boxes'], box_colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    legend_handles = [
        Line2D(
            [0], [0],
            marker='o',
            color='none',
            markerfacecolor='red',
            markeredgecolor='black',
            label='Mean',
        ),
        Line2D([0], [0], color='black', linewidth=1.5, label='Median'),
        Line2D([0], [0], color='black', linestyle='--', linewidth=1.5, label='Whiskers'),
    ]
    ax.legend(
        handles=legend_handles,
        loc='upper left',
        bbox_to_anchor=(1.02, 1.05),
        borderaxespad=0.0,
        fontsize=8,
    )

    ax.set_xlabel(config.xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(f'Search Strategy Comparison — {exp_name}')
    ax.set_ylim(bottom=0)
    ax.set_xticklabels(box_labels, rotation=15, ha='right')
    ax.grid(True)

    plot_path = out_dir / f'comparison_{exp_name}{plot_suffix}.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'  Comparison plot saved to {plot_path}')
    return plot_path


def generate_strategy_comparison_plots(
        strategy_results: dict[str, dict[str, Path]],
        comparison_dir: Path,
        *,
        skip: set[str] | None = None) -> list[Path]:
    """Generate the standard comparison plot set from collected CSV paths."""
    skip = skip or set()
    comparison_dir.mkdir(parents=True, exist_ok=True)
    generated: list[Path] = []

    def _append(plot_path: Path | None) -> None:
        if plot_path is not None:
            generated.append(plot_path)

    def _line_comparison(exp_name: str) -> None:
        csv_by_strategy = strategy_results.get(exp_name, {})
        if exp_name in skip or len(csv_by_strategy) < 2:
            return
        _append(plot_strategy_line_comparison(exp_name, csv_by_strategy, comparison_dir))
        _append(plot_strategy_line_comparison(
            exp_name, csv_by_strategy, comparison_dir,
            y_col='action_count',
            ylabel='Mean Plan Length (number of actions)',
            plot_suffix='_actions'))

    def _camara_box_comparison(exp_name: str) -> None:
        csv_by_strategy = strategy_results.get(exp_name, {})
        if exp_name in skip or len(csv_by_strategy) < 2:
            return
        _append(plot_strategy_box_comparison(
            exp_name, csv_by_strategy, comparison_dir,
            ylabel='Planning Time (seconds)'))
        _append(plot_strategy_box_comparison(
            exp_name, csv_by_strategy, comparison_dir,
            y_col='action_count',
            ylabel='Mean Plan Length (number of actions)',
            plot_suffix='_actions'))
        _append(plot_strategy_box_comparison(
            exp_name, csv_by_strategy, comparison_dir,
            y_col='peak_memory',
            ylabel='Peak Memory (MB)',
            plot_suffix='_memory'))
        _append(plot_strategy_box_comparison(
            exp_name, csv_by_strategy, comparison_dir,
            y_col='comparable_action_count',
            ylabel='Plan Length (original edges)',
            plot_suffix='_comparable_actions'))

    _line_comparison('grid_map')
    _line_comparison('grid_map_no_sas')
    _line_comparison('fd')
    _line_comparison('corridor')
    _line_comparison('ma')
    _line_comparison('ma_no_sas')
    _line_comparison('combined')
    _line_comparison('combined_no_sas')
    _camara_box_comparison('camara_discretized')
    _camara_box_comparison('camara_discretized_no_sas')
    return generated
