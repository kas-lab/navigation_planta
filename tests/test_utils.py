from pathlib import Path

import time

from navigation_planta import reporting
from navigation_planta import utils
from navigation_planta.utils import ExperimentRecord


def test_count_plan_actions_uses_last_numbered_plan(tmp_path: Path) -> None:
    plan_base = tmp_path / "plan"
    (tmp_path / "plan.1").write_text("(move wp0 wp1)\n; cost = 1\n")
    (tmp_path / "plan.2").write_text(
        "(move wp0 wp1)\n(move wp1 wp2)\n; cost = 2\n"
    )

    assert utils.count_plan_actions(plan_base) == 2


def test_count_plan_actions_falls_back_to_plain_plan_file(tmp_path: Path) -> None:
    plan_file = tmp_path / "plan"
    plan_file.write_text("(move wp0 wp1)\n(move wp1 wp2)\n; comment\n")

    assert utils.count_plan_actions(plan_file) == 2


def test_count_plan_actions_returns_no_plan_when_missing(tmp_path: Path) -> None:
    assert utils.count_plan_actions(tmp_path / "missing_plan") == utils.NO_PLAN


def test_run_planner_with_metrics_uses_shared_helpers(
    monkeypatch, tmp_path: Path
) -> None:
    recorded = {}

    def fake_run_subprocess_with_memory(command: list) -> float:
        recorded["command"] = command
        return 12.5

    monkeypatch.setattr(
        utils, "run_subprocess_with_memory", fake_run_subprocess_with_memory
    )
    monkeypatch.setattr(utils, "count_plan_actions", lambda plan_file: 4)

    perf_counter_values = iter([100.0, 100.75])
    monkeypatch.setattr(time, "perf_counter", lambda: next(perf_counter_values))

    plan_file = tmp_path / "plan"
    domain_file = tmp_path / "domain.pddl"
    problem_file = tmp_path / "problem.pddl"

    planning_time, action_count, peak_memory = utils.run_planner_with_metrics(
        plan_file,
        domain_file,
        problem_file,
        "astar(blind())",
    )

    assert planning_time == 0.75
    assert action_count == 4
    assert peak_memory == 12.5
    assert recorded["command"] == [
        "fast-downward.py",
        "--plan-file",
        str(plan_file),
        str(domain_file),
        str(problem_file),
        "--search",
        "astar(blind())",
    ]


def test_save_experiment_records_csv_omits_mode_for_single_mode_runs(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "planning_times.csv"
    records = [
        ExperimentRecord(
            mode="adaptive",
            x_value=10,
            planning_time=0.25,
            action_count=4,
            peak_memory=33.0,
            owltopddl_time=0.1,
        )
    ]

    utils.save_experiment_records_csv(
        csv_path, records, x_name="nodes", time_name="time"
    )

    lines = csv_path.read_text().splitlines()
    assert lines[0] == "nodes,owltopddl_time,time,action_count,peak_memory"
    assert lines[1].startswith("10,1.000000000000000056e-01,")


def test_save_experiment_records_csv_keeps_mode_for_multi_mode_runs(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "planning_times.csv"
    records = [
        ExperimentRecord(
            mode="adaptive",
            x_value="wp1_wp2",
            planning_time=0.25,
            action_count=4,
            peak_memory=33.0,
        ),
        ExperimentRecord(
            mode="no-adaptation",
            x_value="wp1_wp2",
            planning_time=0.5,
            action_count=5,
            peak_memory=30.0,
        ),
    ]

    utils.save_experiment_records_csv(csv_path, records, x_name="init_goal")

    lines = csv_path.read_text().splitlines()
    assert lines[0] == "mode,init_goal,planning_time,action_count,peak_memory"
    assert lines[1].startswith("adaptive,wp1_wp2,")
    assert lines[2].startswith("no-adaptation,wp1_wp2,")


def test_plot_memory_summary_creates_png(tmp_path: Path) -> None:
    records = [
        ExperimentRecord("adaptive", 10, 0.1, 3, 40.0),
        ExperimentRecord("adaptive", 20, 0.2, 4, 42.0),
        ExperimentRecord("no-adaptation", 10, 0.1, 3, 35.0),
        ExperimentRecord("no-adaptation", 20, 0.2, 4, 36.0),
    ]
    out_file = tmp_path / "peak_memory_boxplot.png"

    reporting.plot_memory_summary(
        tmp_path,
        records,
        {"adaptive": "Adaptive", "no-adaptation": "No adaptation"},
        filename=out_file.name,
    )

    assert out_file.exists()
    assert out_file.stat().st_size > 0


def test_plot_mode_summary_creates_png(tmp_path: Path) -> None:
    records = [
        ExperimentRecord("adaptive", "wp1_wp2", 0.4, 3, 40.0),
        ExperimentRecord("adaptive", "wp2_wp3", 0.5, 4, 42.0),
        ExperimentRecord("no-adaptation", "wp1_wp2", 0.7, 5, 35.0),
        ExperimentRecord("no-adaptation", "wp2_wp3", 0.8, 6, 36.0),
    ]
    out_file = tmp_path / "camara_summary.png"

    reporting.plot_mode_summary(
        tmp_path,
        records,
        {"adaptive": "Adaptive", "no-adaptation": "No adaptation"},
        filename=out_file.name,
    )

    assert out_file.exists()
    assert out_file.stat().st_size > 0
