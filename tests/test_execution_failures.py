"""Regressions for failed invocations, stale artifacts and baseline metrics."""

import importlib
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import networkx as nx
import pytest

from helpers import experiment_runner, utils


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def runners(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "scripts"))
    return (
        importlib.import_module("run_camara_scenario_discretized"),
        importlib.import_module("run_camara_prism_scenario"),
    )


def test_baseline_metrics_do_not_invent_a_configuration(tmp_path):
    graph = nx.Graph()
    graph.add_edge(0, 1, weight=10.0)
    plan = tmp_path / "plan"
    plan.write_text("(move wp0 wp2)\n(move wp2 wp1)\n; cost = 2\n")
    assert utils.count_comparable_plan_actions_and_energy(
        graph, plan, {0, 1}, 32560, calculate_energy=False
    ) == (1, None, 10.0)


def _split_corridor_graphs():
    graph = nx.Graph()
    graph.add_edge(0, 1, weight=10.0)
    segment_graph = nx.Graph()
    segment_graph.add_edge(0, 2, weight=5.0)
    segment_graph.add_edge(2, 1, weight=5.0)
    return graph, segment_graph


def test_adaptive_energy_metrics_are_preserved(tmp_path):
    graph, segment_graph = _split_corridor_graphs()
    plan = tmp_path / "plan"
    plan.write_text("(reconfigure f_localization fd_aruco)\n(move wp0 wp2)\n(move wp2 wp1)\n")
    assert utils.count_comparable_plan_actions_and_energy(
        graph, plan, {0, 1}, 32560, segment_graph=segment_graph
    ) == (2, 32560 - utils.energy_cost(10.0, utils.CONFIG_ENERGY["aruco"]), 10.0)


def test_constant_configuration_corridor_truncates_once_like_prism(tmp_path):
    # Three 10/3 m segments: truncating per segment would charge 1422 instead
    # of PRISM's single truncation over the whole corridor (1424).
    graph = nx.Graph()
    graph.add_edge(0, 1, weight=10.0)
    segment_graph = nx.Graph()
    for u, v in [(0, 2), (2, 3), (3, 1)]:
        segment_graph.add_edge(u, v, weight=10.0 / 3)
    plan = tmp_path / "plan"
    plan.write_text(
        "(reconfigure f_localization fd_mprt_lidar)\n"
        "(move wp0 wp2)\n(move wp2 wp3)\n(move wp3 wp1)\n")
    _, remaining, _ = utils.count_comparable_plan_actions_and_energy(
        graph, plan, {0, 1}, 32560, segment_graph=segment_graph)
    assert remaining == 32560 - utils.energy_cost(10.0, utils.CONFIG_ENERGY["mprt_lidar"])


def test_mid_corridor_reconfiguration_charges_each_segment_to_its_configuration(tmp_path):
    graph, segment_graph = _split_corridor_graphs()
    plan = tmp_path / "plan"
    plan.write_text(
        "(reconfigure f_localization fd_aruco)\n(move wp0 wp2)\n"
        "(reconfigure f_localization fd_aruco_headlamp)\n(move wp2 wp1)\n")
    count, remaining, distance = utils.count_comparable_plan_actions_and_energy(
        graph, plan, {0, 1}, 32560, segment_graph=segment_graph)
    expected_cost = int(
        utils.unrounded_energy_cost(5.0, utils.CONFIG_ENERGY["aruco"])
        + utils.unrounded_energy_cost(5.0, utils.CONFIG_ENERGY["aruco_headlamp"]))
    assert (count, remaining, distance) == (3, 32560 - expected_cost, 10.0)
    # Charging the whole corridor to the final configuration was the old bug.
    assert remaining != 32560 - utils.energy_cost(10.0, utils.CONFIG_ENERGY["aruco_headlamp"])


def test_plan_resolver_selects_highest_numbered_plan(tmp_path):
    plan = tmp_path / "plan"
    for filename in ["plan", "plan.9", "plan.10"]:
        (tmp_path / filename).write_text("")
    assert utils._resolve_fd_plan_file(plan) == tmp_path / "plan.10"


def test_plan_resolver_ignores_non_numeric_suffixes(tmp_path):
    plan = tmp_path / "plan"
    for filename in ["plan", "plan.log", "plan.1.bak"]:
        (tmp_path / filename).write_text("")
    assert utils._resolve_fd_plan_file(plan) == plan
    plan.unlink()
    assert utils._resolve_fd_plan_file(plan) is None


def test_plan_resolver_falls_back_to_unnumbered_plan(tmp_path):
    plan = tmp_path / "plan"
    plan.write_text("")
    assert utils._resolve_fd_plan_file(plan) == plan


def test_failed_planner_clears_old_plans_and_does_not_parse_them(monkeypatch, tmp_path):
    plan = tmp_path / "plan"
    for filename in ["plan", "plan.1", "plan.10"]:
        (tmp_path / filename).write_text("(move wp0 wp1)\n")
    original_runner = utils.run_subprocess_with_memory

    def fail(command, **kwargs):
        assert not plan.exists()
        assert not list(tmp_path.glob("plan.*"))
        return original_runner(
            [sys.executable, "-c", 'import sys; print("bad input", file=sys.stderr); sys.exit(2)'],
            **kwargs,
        )

    monkeypatch.setattr(utils, "run_subprocess_with_memory", fail)
    with pytest.raises(subprocess.CalledProcessError) as error:
        utils.run_planner_with_metrics(plan, tmp_path / "domain", tmp_path / "problem", "astar(blind())")
    assert error.value.returncode == 2
    assert "bad input" in (tmp_path / "planner.log").read_text()


def test_planner_execution_error_reaches_retry_layer(monkeypatch, tmp_path):
    attempts = 0

    def run_command(command, **kwargs):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise subprocess.CalledProcessError(23, command)
        Path(command[2]).write_text("(move wp0 wp1)\n")
        return 10.0

    def prepare(folder, mode):
        problem = folder / "problem.pddl"
        problem.write_text("placeholder")
        return problem

    monkeypatch.setattr(utils, "run_subprocess_with_memory", run_command)
    record = experiment_runner.run_with_retries(
        lambda: experiment_runner.execute_pddl_case(
            folder=tmp_path, mode="no-adaptation", case_id="case", x_value=1,
            search="astar(blind())", prepare_problem=prepare,
            adaptive_preprocessor=None, baseline_domain=tmp_path / "domain.pddl",
        ), attempts=2, context="execution-failure-test",
    )
    assert attempts == 2
    assert record.action_count == 1


@pytest.mark.parametrize("code", [11, 12])
def test_unsolved_search_is_recorded_without_stale_plan(monkeypatch, tmp_path, code):
    plan = tmp_path / "plan"
    plan.write_text("(move wp0 wp1)\n")
    original_runner = utils.run_subprocess_with_memory
    monkeypatch.setattr(
        utils, "run_subprocess_with_memory",
        lambda command, **kwargs: original_runner(
            [sys.executable, "-c", f"raise SystemExit({code})"], **kwargs),
    )
    _, actions, moves, reconfigs, _ = utils.run_planner_with_metrics(
        plan, tmp_path / "domain", tmp_path / "problem", "astar(blind())")
    assert (actions, moves, reconfigs) == (-1, -1, -1)


@pytest.mark.parametrize("exit_error", [False, True])
def test_prism_never_consumes_stale_strategy(runners, monkeypatch, tmp_path, exit_error):
    _, prism_runner = runners
    folder = tmp_path / "wp1_wp2_0"
    folder.mkdir()
    strategy = folder / "strategy.txt"
    strategy.write_text("old strategy")
    generator = SimpleNamespace(
        find_shortest_path=lambda **kw: [1, 2],
        generate_prism_model=lambda *args: None,
    )

    def run(command, **kwargs):
        assert not strategy.exists()
        assert kwargs["log_file"] == folder / "prism.log"
        if exit_error:
            raise subprocess.CalledProcessError(1, command)
        return 1.0  # A successful exit without an export is also an error.

    monkeypatch.setattr(prism_runner, "run_subprocess_with_memory", run)
    error = subprocess.CalledProcessError if exit_error else RuntimeError
    with pytest.raises(error):
        prism_runner.run(tmp_path, 0, 1, 2, generator)
    assert not strategy.exists()


@pytest.mark.skipif(not shutil.which("fast-downward.py"), reason="Fast Downward required")
def test_real_discretized_baseline_returns_metrics(runners, tmp_path):
    discretized_runner, _ = runners
    record = discretized_runner.run_single_case(tmp_path, "no-adaptation", 0, (12, 11))
    assert record.action_count == 11
    assert record.move_action_count == 11
    assert record.reconfig_action_count == 0
    assert record.comparable_action_count == 1
    assert record.distance > 0
    assert record.remaining_battery is None
    assert (tmp_path / "no-adaptation/wp12_wp11_0/planner.log").is_file()


@pytest.mark.skipif(not shutil.which("fast-downward.py"), reason="Fast Downward required")
def test_real_unsolvable_problem_is_not_an_execution_error(tmp_path):
    from helpers.no_adaptation import NoAdaptationMapGenerator

    generator = NoAdaptationMapGenerator()
    for node in range(4):
        generator.graph.add_node(node, pos=(float(node), 0.0))
    generator.graph.add_edge(0, 1, weight=1.0)
    generator.graph.add_edge(2, 3, weight=1.0)
    problem = tmp_path / "problem.pddl"
    generator.generate_domain_problem_files(problem_filename=problem, init_goal=(0, 3))
    _, actions, moves, reconfigs, _ = utils.run_planner_with_metrics(
        tmp_path / "plan", ROOT / "pddl/domain_no_sas.pddl", problem, "astar(blind())")
    assert (actions, moves, reconfigs) == (-1, -1, -1)
    assert (tmp_path / "planner.log").is_file()


@pytest.mark.skipif(not shutil.which("fast-downward.py"), reason="Fast Downward required")
def test_real_missing_inputs_cannot_reuse_stale_plan(tmp_path):
    plan = tmp_path / "plan"
    plan.write_text("(move wp0 wp1)\n")
    with pytest.raises(subprocess.CalledProcessError):
        utils.run_planner_with_metrics(
            plan, tmp_path / "missing-domain.pddl", tmp_path / "missing-problem.pddl", "astar(blind())")
    assert not plan.exists()
    assert (tmp_path / "planner.log").stat().st_size > 0
