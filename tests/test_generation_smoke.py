from pathlib import Path

import networkx as nx
import pytest


try:
    from navigation_planta.map_generator import MapGenerator
    from navigation_planta.scenario_variants import MissionActionNoAdaptationMapGenerator
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


def test_write_problem_fast_serializes_core_problem_sections(tmp_path: Path) -> None:
    generator = MapGenerator()
    generator.graph = build_sample_graph()
    generator.generate_domain()
    generator.generate_problem()

    problem_file = tmp_path / "problem_raw.pddl"
    generator.write_problem_fast(problem_file)
    text = problem_file.read_text()

    assert problem_file.exists()
    assert "(:objects" in text
    assert "wp0" in text
    assert "wp1" in text
    assert "wp2" in text
    assert "robot_at wp0" in text
    assert "corridor wp0 wp1" in text
    assert "robot_at wp2" in text


def test_generate_domain_problem_files_rewrites_decimal_names(tmp_path: Path) -> None:
    generator = MapGenerator()
    generator.graph = build_sample_graph()

    problem_file = tmp_path / "problem.pddl"
    generator.generate_domain_problem_files(problem_filename=problem_file)
    text = problem_file.read_text()

    assert problem_file.exists()
    assert "o_0_0_decimal" not in text
    assert "o_0_8_decimal" not in text
    assert "o_1_0_decimal" not in text
    assert "0.0_decimal" in text
    assert "0.8_decimal" in text
    assert "1.0_decimal" in text


def test_mission_no_adaptation_problem_generation_smoke(tmp_path: Path) -> None:
    generator = MissionActionNoAdaptationMapGenerator(
        num_nodes=3,
        nodes_skip=0.0,
        unconnected_amount=0.0,
        unsafe_amount=0.0,
        dark_amount=0.0,
        m=3,
    )
    generator.graph = build_sample_graph()

    problem_file = tmp_path / "problem_no_sas_ma.pddl"
    generator.generate_domain_problem_files(problem_filename=problem_file)
    text = problem_file.read_text()

    assert problem_file.exists()
    assert "inspection_waypoint" in text
    assert "delivery_waypoint" in text
    assert "inspection_done" in text
    assert "delivery_done" in text
