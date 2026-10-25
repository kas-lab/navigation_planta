"""Known-value checks independent of the manuscript checkout and planners."""
import math

import pytest

from paper_statistics import (
    box_statistics, comparable_moves, describe, rank_sample, sweep_statistics,
)


def test_sample_not_population_sd():
    stats = describe([1, 2, 3])
    assert stats['mean'] == 2
    assert stats['std'] == 1
    assert stats['q1'] == 1.5
    assert stats['median'] == 2
    assert stats['q3'] == 2.5
    with pytest.raises(ValueError):
        describe([1])
    with pytest.raises(ValueError):
        describe([1, math.inf])


def test_tukey_constant_values_and_boundary():
    stats = box_statistics([4] * 10)
    assert stats['lower_whisker'] == stats['upper_whisker'] == 4
    assert stats['outlier_count'] == 0
    # Q1=0 Q3=2: 5 is on the upper fence, not beyond it.
    stats = box_statistics([0, 0, 0, 1, 2, 2, 5])
    assert stats['upper_whisker'] == 5
    assert stats['outlier_count'] == 0


def test_outlier_multiplicity_and_sampling():
    stats = box_statistics([0] * 1000 + [10] * 300 + [20] * 10)
    assert stats['outlier_count'] == 310
    assert len(stats['outlier_marks']) == 250
    assert stats['outlier_marks'][0] == 10
    assert stats['outlier_marks'][-1] == 20
    assert stats['outlier_marks'].count(10) > 200
    assert rank_sample([1, 2, 3], 0) == []
    assert rank_sample([], 20) == []
    assert rank_sample([1, 2, 3], 1) == [2]
    assert rank_sample([1, 2], 5) == [1, 2]


def test_move_only_metric():
    row = dict(move_action=30, comparable_action_count=7, reconfig_action=2)
    assert comparable_moves(row, 'eager') == 5
    assert comparable_moves(row, 'astar') == 5
    assert comparable_moves(row, 'prism') == 30


def test_processed_columns_full_precision_and_numeric_sort():
    rows = [dict(nodes=x, time=t, action_count=t * 2, peak_memory=10)
            for x in (20, 9) for t in (1, 3)]
    processed, summaries = sweep_statistics(rows, 'nodes', 'time')
    assert [r['nodes'] for r in processed] == [9, 20]
    row = processed[0]
    assert list(row) == ['nodes', 'mean_time', 'std_time', 'mean_action_count',
                         'std_action_count', 'mean_time_plus_std', 'mean_time_minus_std',
                         'mean_action_count_plus_std', 'mean_action_count_minus_std',
                         'mean_peak_memory', 'std_peak_memory',
                         'mean_peak_memory_plus_std', 'mean_peak_memory_minus_std']
    assert row['std_time'] == pytest.approx(math.sqrt(2))
    assert row['mean_time_plus_std'] == pytest.approx(2 + math.sqrt(2))
    assert summaries[0]['metrics']['peak_memory']['std'] == 0
    assert row['mean_peak_memory'] == row['mean_peak_memory_plus_std'] == row['mean_peak_memory_minus_std'] == 10
    assert row['std_peak_memory'] == 0


def test_memory_mean_and_sample_sd_by_parameter():
    rows = [dict(nodes=x, time=1, action_count=2, peak_memory=m)
            for x in (9, 20) for m in (10, 14)]
    processed, _ = sweep_statistics(rows, 'nodes', 'time')
    for row in processed:
        assert row['mean_peak_memory'] == 12
        assert row['std_peak_memory'] == pytest.approx(math.sqrt(8))
        assert row['mean_peak_memory_plus_std'] == pytest.approx(12 + math.sqrt(8))
        assert row['mean_peak_memory_minus_std'] == pytest.approx(12 - math.sqrt(8))
