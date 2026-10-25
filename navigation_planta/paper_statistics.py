"""Pure, planner-independent statistics for publication artifacts.

Sample SD describes experimental variability, not uncertainty of the mean.
Quartiles and outlier sampling match the manuscript's legacy preprocessing.
"""

import math
from statistics import mean, stdev


def rank_sample(sorted_values, max_marks=250):
    """Sample ranks deterministically, preserving duplicate observations."""
    n = len(sorted_values)
    if max_marks <= 0 or not n:
        return []
    if n <= max_marks:
        return list(sorted_values)
    if max_marks == 1:
        return [sorted_values[n // 2]]
    indices = sorted({round(i * (n - 1) / (max_marks - 1)) for i in range(max_marks)})
    return [sorted_values[i] for i in indices]


def quantile(sorted_values, probability):
    """Type-7 (linear interpolated) quantile of a nonempty sorted sequence."""
    position = (len(sorted_values) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    return sorted_values[lower] + (position - lower) * (
        sorted_values[upper] - sorted_values[lower])


def describe(values):
    """Return full-precision descriptive statistics; require sample SD."""
    values = sorted(float(value) for value in values)
    if len(values) < 2 or not all(math.isfinite(value) for value in values):
        raise ValueError('Statistics require at least two finite observations')
    return {
        'n': len(values), 'mean': mean(values), 'std': stdev(values),
        'min': values[0], 'q1': quantile(values, .25),
        'median': quantile(values, .5), 'q3': quantile(values, .75),
        'max': values[-1],
    }


def box_statistics(values, max_marks=250):
    values = sorted(float(value) for value in values)
    stats = describe(values)
    iqr = stats['q3'] - stats['q1']
    low, high = stats['q1'] - 1.5 * iqr, stats['q3'] + 1.5 * iqr
    inliers = [value for value in values if low <= value <= high]
    outliers = [value for value in values if value < low or value > high]
    stats.update(lower_whisker=inliers[0], upper_whisker=inliers[-1],
                 outlier_count=len(outliers), outlier_marks=rank_sample(outliers, max_marks))
    return stats


def comparable_moves(row, method):
    if method == 'prism':
        return row['move_action']
    return row['comparable_action_count'] - row['reconfig_action']


def sweep_statistics(rows, group, time):
    """Return legacy processed columns plus full per-group metric summaries."""
    processed, summaries = [], []
    for x in sorted({row[group] for row in rows}):
        subset = [row for row in rows if row[group] == x]
        metrics = {name: describe(row[column] for row in subset)
                   for name, column in [('time', time), ('action_count', 'action_count'),
                                        ('peak_memory', 'peak_memory')]}
        t, a = metrics['time'], metrics['action_count']
        record = {group: x, 'mean_time': t['mean'], 'std_time': t['std'],
                  'mean_action_count': a['mean'], 'std_action_count': a['std']}
        for name in ('time', 'action_count'):
            record[f'mean_{name}_plus_std'] = record[f'mean_{name}'] + record[f'std_{name}']
            record[f'mean_{name}_minus_std'] = record[f'mean_{name}'] - record[f'std_{name}']
        memory = metrics['peak_memory']
        record.update(mean_peak_memory=memory['mean'], std_peak_memory=memory['std'],
                      mean_peak_memory_plus_std=memory['mean'] + memory['std'],
                      mean_peak_memory_minus_std=memory['mean'] - memory['std'])
        processed.append(record)
        summaries.append({'x': x, 'metrics': metrics})
    return processed, summaries
