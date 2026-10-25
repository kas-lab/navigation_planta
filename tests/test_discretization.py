"""Regression tests for semantic preservation when splitting corridors."""

import math
from pathlib import Path
import shutil
import subprocess

import networkx as nx
import pytest

from helpers.map_io import MapIOPlottingMixin


MAP_FILE = Path(__file__).resolve().parents[1] / "data/map_camara_2020_paper/map-p2cp3.json"


class MapLoader(MapIOPlottingMixin):
    def __init__(self):
        self.graph = nx.Graph()


def replacement_path(generator, u, v):
    # Exclude other original waypoints so that this is the replacement of this
    # particular corridor, not a shortcut through the rest of the map.
    intermediate = set(generator.graph) - set(generator.original_graph)
    return nx.shortest_path(generator.graph.subgraph(intermediate | {u, v}), u, v)


@pytest.mark.parametrize("dark,unsafe", [(False, False), (True, False), (False, True), (True, True)])
def test_split_preserves_attributes_and_total_length(dark, unsafe):
    generator = MapLoader()
    for node, pos in enumerate([(0, 0), (1, 0), (3.5, 0)]):
        generator.graph.add_node(node, pos=pos)
    hitrate = {"from": "l1", "to": "l2", "configuration": {"hitrate": 0.2}}
    generator.graph.add_edge(0, 1, weight=1.0, dark=False, unsafe=False)
    generator.graph.add_edge(1, 2, weight=2.5, dark=dark, unsafe=unsafe,
                             hitrate=hitrate, label="original corridor")
    before = generator.graph.copy()

    generator.discretize_graph()

    assert nx.utils.graphs_equal(generator.original_graph, before)
    assert generator.graph[0][1] == before[0][1]
    assert not generator.graph.has_edge(1, 2)
    path = replacement_path(generator, 1, 2)
    assert len(path) == 4
    lengths = []
    for u, v in zip(path, path[1:]):
        data = generator.graph[u][v]
        assert data["dark"] is dark
        assert data["unsafe"] is unsafe
        assert data["hitrate"] == hitrate
        assert data["label"] == "original corridor"
        length = math.dist(generator.graph.nodes[u]["pos"], generator.graph.nodes[v]["pos"])
        assert data["weight"] == pytest.approx(length)
        assert length <= 1.0
        lengths.append(data["weight"])
    assert sum(lengths) == pytest.approx(2.5)
    assert nx.is_connected(generator.graph)


def test_camara_all_corridors_keep_metadata_and_connectivity():
    generator = MapLoader()
    generator.load_and_discretize_json(MAP_FILE)
    original = generator.original_graph
    assert sum(d["dark"] or d["unsafe"] for _, _, d in original.edges(data=True)) == 6
    limit = min(d["weight"] for _, _, d in original.edges(data=True))

    for u, v, original_data in original.edges(data=True):
        path = replacement_path(generator, u, v)
        total_length = 0.0
        for a, b in zip(path, path[1:]):
            data = generator.graph[a][b]
            assert {k: value for k, value in data.items() if k != "weight"} == {
                k: value for k, value in original_data.items() if k != "weight"
            }
            assert data["weight"] <= limit + 1e-9
            total_length += data["weight"]
        assert total_length == pytest.approx(original_data["weight"])

    assert nx.number_connected_components(generator.graph) == nx.number_connected_components(original)
    for component in nx.connected_components(original):
        root = next(iter(component))
        assert component <= nx.node_connected_component(generator.graph, root)


def test_camara_discretized_requirements_reach_pddl(tmp_path):
    pytest.importorskip("unified_planning")
    from helpers.map_generator import MapGenerator

    generator = MapGenerator()
    generator.load_and_discretize_json(MAP_FILE)
    problem_file = tmp_path / "problem.pddl"
    generator.generate_domain_problem_files(problem_filename=problem_file, init_goal=(12, 11))
    text = problem_file.read_text()

    for u, v, data in generator.graph.edges(data=True):
        light = "1.0_decimal" if data["dark"] else "0.0_decimal"
        safety = "0.8_decimal" if data["unsafe"] else "0.0_decimal"
        for a, b in [(u, v), (v, u)]:
            assert f"(corridor wp{a} wp{b})" in text
            assert f"(light_requirement wp{a} wp{b} {light})" in text
            assert f"(safety_requirement wp{a} wp{b} {safety})" in text


@pytest.mark.skipif(
    not shutil.which("OWLToPDDL.sh") or not shutil.which("fast-downward.py"),
    reason="OWL-to-PDDL and Fast Downward executables are required",
)
def test_discretized_camara_corridor_can_be_planned_with_required_configuration(tmp_path):
    pytest.importorskip("unified_planning")
    from helpers.map_generator import MapGenerator

    root = MAP_FILE.parents[2]
    generator = MapGenerator()
    generator.load_and_discretize_json(MAP_FILE)
    problem = tmp_path / "problem.pddl"
    domain_created = tmp_path / "domain_created.pddl"
    problem_created = tmp_path / "problem_created.pddl"
    plan = tmp_path / "plan"
    generator.generate_domain_problem_files(problem_filename=problem, init_goal=(12, 11))
    commands = [
        ["OWLToPDDL.sh", f"--owl={root / 'owl/navigation.owl'}", "--tBox",
         f"--inDomain={root / 'pddl/domain_sas.pddl'}", f"--outDomain={domain_created}",
         "--aBox", f"--inProblem={problem}", f"--outProblem={problem_created}",
         "--replace-output", "--add-num-comparisons"],
        ["fast-downward.py", "--plan-file", str(plan), str(domain_created),
         str(problem_created), "--search", "astar(blind())"],
    ]
    for command in commands:
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout + result.stderr

    actions = [line for line in plan.read_text().splitlines() if line.startswith("(")]
    assert actions[0] == "(reconfigure f_localization fd_aruco_headlamp)"
    path = replacement_path(generator, 12, 11)
    assert actions[1:] == [
        f"(move wp{u} wp{v} 0.8_decimal 1.0_decimal)"
        for u, v in zip(path, path[1:])
    ]
