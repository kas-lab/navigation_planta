import csv
from pathlib import Path

from navigation_planta.reporting import (
    plot_numeric_sweep_results,
    plot_strategy_box_comparison,
    plot_strategy_line_comparison,
    summarize_mode_records,
    write_metric_summary_csv,
    write_summary_report,
)
from navigation_planta.utils import ExperimentRecord, save_experiment_records_csv


def test_plot_numeric_sweep_results_handles_missing_action_series(tmp_path: Path) -> None:
    records = [
        ExperimentRecord("adaptive", 10, 0.2, -1, -1, -1, 40.0),
        ExperimentRecord("adaptive", 20, 0.3, -1, -1, -1, 41.0),
        ExperimentRecord("no-adaptation", 10, 0.4, -1, -1, -1, 35.0),
        ExperimentRecord("no-adaptation", 20, 0.6, -1, -1, -1, 36.0),
    ]

    plot_numeric_sweep_results(
        tmp_path,
        records,
        {"adaptive": "Adaptive", "no-adaptation": "No adaptation"},
        xlabel="Nodes",
        time_title="Time",
        action_title="Actions",
        filename="numeric_sweep.png",
        x_values=[10, 20],
    )

    out_file = tmp_path / "numeric_sweep.png"
    assert out_file.exists()
    assert out_file.stat().st_size > 0


def test_summarize_mode_records_and_report_file(tmp_path: Path) -> None:
    records = [
        ExperimentRecord("adaptive", "wp1_wp2", 0.4, 3, 3, 0, 40.0),
        ExperimentRecord("adaptive", "wp2_wp3", 0.6, 5, 4, 1, 42.0),
    ]

    lines = summarize_mode_records(records, {"adaptive": "Adaptive"})
    assert len(lines) == 1
    assert "Mean" in lines[0]
    assert "Max mem" in lines[0]

    report_path = write_summary_report(tmp_path, lines)
    assert report_path == tmp_path / "summary.txt"
    assert report_path.read_text() == lines[0] + "\n"


def test_write_metric_summary_csv(tmp_path: Path) -> None:
    records = [
        ExperimentRecord("adaptive", "wp1_wp2", 0.4, 4, 4, 0, 40.0,
                         remaining_battery=100, distance=12.0),
        ExperimentRecord("adaptive", "wp2_wp3", 0.6, 6, 5, 1, 42.0,
                         remaining_battery=80, distance=18.0),
        ExperimentRecord("adaptive", "wp3_wp4", 0.5, -1, -1, -1, 41.0,
                         remaining_battery=-1, distance=None),
    ]

    report_path = write_metric_summary_csv(
        tmp_path, records, {"adaptive": "Adaptive"})
    assert report_path == tmp_path / "summary.csv"

    rows = list(csv.reader(report_path.read_text().splitlines()))
    header, data = rows[0], rows[1]
    assert header[:2] == ["mode", "n"]
    assert "move_action_mean" in header
    assert "reconfig_action_std" in header
    assert "distance_std" in header

    row = dict(zip(header, data))
    assert row["mode"] == "Adaptive"
    assert row["n"] == "3"
    # -1 no-plan sentinel dropped: mean over {4, 5}
    assert float(row["move_action_mean"]) == 4.5
    # peak_memory keeps every sample
    assert float(row["peak_memory_mean"]) == (40.0 + 42.0 + 41.0) / 3
    # None dropped: mean over {12.0, 18.0}
    assert float(row["distance_mean"]) == 15.0
    # -1 dropped: mean over {100, 80}
    assert float(row["remaining_battery_mean"]) == 90.0


def test_write_metric_summary_csv_reports_nan_for_absent_metric(tmp_path: Path) -> None:
    records = [
        ExperimentRecord("adaptive", "wp1_wp2", 0.4, 4, 4, 0, 40.0),
        ExperimentRecord("adaptive", "wp2_wp3", 0.6, 6, 5, 1, 42.0),
    ]

    report_path = write_metric_summary_csv(
        tmp_path, records, {"adaptive": "Adaptive"})
    header, data = list(csv.reader(report_path.read_text().splitlines()))
    row = dict(zip(header, data))

    # remaining_battery / distance never set on these records
    assert row["remaining_battery_mean"] == "nan"
    assert row["remaining_battery_std"] == "nan"
    assert row["distance_mean"] == "nan"


def test_plot_strategy_line_comparison_creates_expected_filename(tmp_path: Path) -> None:
    csv_a = tmp_path / "fd_a.csv"
    csv_b = tmp_path / "fd_b.csv"
    save_experiment_records_csv(
        csv_a,
        [
            ExperimentRecord("adaptive", 2, 0.2, 3, 3, 0, 40.0),
            ExperimentRecord("adaptive", 4, 0.3, 4, 4, 0, 41.0),
        ],
        x_name="n_fd",
    )
    save_experiment_records_csv(
        csv_b,
        [
            ExperimentRecord("adaptive", 2, 0.4, 5, 5, 0, 35.0),
            ExperimentRecord("adaptive", 4, 0.5, 6, 6, 0, 36.0),
        ],
        x_name="n_fd",
    )

    plot_strategy_line_comparison(
        "fd",
        {"astar(blind())": csv_a, "eager_greedy([goalcount()])": csv_b},
        tmp_path,
        y_col="action_count",
        ylabel="Mean Plan Length (actions)",
        plot_suffix="_actions",
    )

    out_file = tmp_path / "comparison_fd_actions.png"
    assert out_file.exists()
    assert out_file.stat().st_size > 0


def test_plot_strategy_box_comparison_creates_expected_filename(tmp_path: Path) -> None:
    csv_a = tmp_path / "camara_a.csv"
    csv_b = tmp_path / "camara_b.csv"
    save_experiment_records_csv(
        csv_a,
        [
            ExperimentRecord("adaptive", "wp1_wp2", 0.2, 3, 3, 0, 40.0),
            ExperimentRecord("adaptive", "wp2_wp3", 0.3, 4, 4, 0, 41.0),
        ],
        x_name="init_goal",
        time_name="time",
    )
    save_experiment_records_csv(
        csv_b,
        [
            ExperimentRecord("adaptive", "wp1_wp2", 0.5, 5, 5, 0, 35.0),
            ExperimentRecord("adaptive", "wp2_wp3", 0.7, 6, 6, 0, 36.0),
        ],
        x_name="init_goal",
        time_name="time",
    )

    plot_strategy_box_comparison(
        "camara_discretized",
        {"astar(blind())": csv_a, "prism (baseline)": csv_b},
        tmp_path,
        y_col="peak_memory",
        ylabel="Peak Memory (MB)",
        plot_suffix="_memory",
    )

    out_file = tmp_path / "comparison_camara_discretized_memory.png"
    assert out_file.exists()
    assert out_file.stat().st_size > 0
