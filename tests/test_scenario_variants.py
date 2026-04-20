from pathlib import Path

import networkx as nx
import pytest


try:
    from navigation_planta.scenario_variants import (
        CombinedScenarioMapGenerator,
        CorridorTypeMapGenerator,
        MissionActionMapGenerator,
    )
except Exception as exc:  # pragma: no cover - environment dependent
    pytest.skip(
        f"Unified Planning stack is not importable in this host environment: {exc}",
        allow_module_level=True,
    )


def build_sample_graph() -> nx.Graph:
    graph = nx.Graph()
    graph.add_node(0, pos=(0.0, 0.0))
    graph.add_node(1, pos=(1.0, 0.0))
    graph.add_node(2, pos=(2.0, 0.0))
    graph.add_edge(0, 1, weight=1.0, dark=False, unsafe=False)
    graph.add_edge(1, 2, weight=1.0, dark=True, unsafe=True)
    return graph


def test_corridor_type_variant_emits_extra_corridor_requirements(
    tmp_path: Path,
) -> None:
    generator = CorridorTypeMapGenerator(
        num_nodes=3,
        nodes_skip=0.0,
        unconnected_amount=0.0,
        unsafe_amount=0.0,
        dark_amount=0.0,
        outdoor_amount=1.0,
        dusty_amount=1.0,
    )
    generator.graph = build_sample_graph()

    problem_file = tmp_path / "corridor_variant_problem.pddl"
    generator.generate_domain_problem_files(problem_filename=problem_file)
    text = problem_file.read_text()

    assert "outdoor_requirement" in text
    assert "dust_requirement" in text


def test_mission_action_variant_emits_waypoints_and_done_goals(
    tmp_path: Path,
) -> None:
    generator = MissionActionMapGenerator(
        num_nodes=3,
        nodes_skip=0.0,
        unconnected_amount=0.0,
        unsafe_amount=0.0,
        dark_amount=0.0,
        m=3,
    )
    generator.graph = build_sample_graph()

    problem_file = tmp_path / "mission_variant_problem.pddl"
    generator.generate_domain_problem_files(problem_filename=problem_file)
    text = problem_file.read_text()

    assert "inspection_waypoint" in text
    assert "delivery_waypoint" in text
    assert "inspection_done" in text
    assert "delivery_done" in text


def test_combined_variant_emits_corridor_and_mission_extensions(
    tmp_path: Path,
) -> None:
    generator = CombinedScenarioMapGenerator(
        num_nodes=3,
        nodes_skip=0.0,
        unconnected_amount=0.0,
        unsafe_amount=0.0,
        dark_amount=0.0,
        outdoor_amount=1.0,
        dusty_amount=1.0,
    )
    generator.graph = build_sample_graph()

    problem_file = tmp_path / "combined_variant_problem.pddl"
    generator.generate_domain_problem_files(problem_filename=problem_file)
    text = problem_file.read_text()

    assert "outdoor_requirement" in text
    assert "dust_requirement" in text
    assert "inspection_waypoint" in text
    assert "report_done" in text
