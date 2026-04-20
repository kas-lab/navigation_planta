import random

import networkx as nx
import numpy as np

from navigation_planta.graph_generation import GraphGenerationMixin
from navigation_planta.map_io import MapIOPlottingMixin


class DummyMapGenerator(GraphGenerationMixin, MapIOPlottingMixin):
    def __init__(
        self,
        *,
        num_nodes: int = 9,
        nodes_skip: float = 0.0,
        unconnected_amount: float = 0.0,
        unsafe_amount: float = 0.0,
        dark_amount: float = 0.0,
    ) -> None:
        self.graph = nx.Graph()
        self.num_nodes = num_nodes
        self.nodes_skip = nodes_skip
        self.unconnected_amount = unconnected_amount
        self.unsafe_amount = unsafe_amount
        self.dark_amount = dark_amount


def test_generate_grid_graph_returns_connected_graph_with_edge_metadata() -> None:
    random.seed(7)

    generator = DummyMapGenerator(
        num_nodes=9,
        nodes_skip=0.0,
        unconnected_amount=0.0,
    )
    graph = generator.generate_grid_graph()

    assert graph.number_of_nodes() == 9
    assert nx.is_connected(graph)

    for _, _, edge_data in graph.edges(data=True):
        assert edge_data["weight"] == 10.0
        assert edge_data["dark"] is False
        assert edge_data["unsafe"] is False


def test_generate_connected_grid_map_assigns_dark_and_unsafe_flags() -> None:
    random.seed(11)

    generator = DummyMapGenerator(
        num_nodes=9,
        nodes_skip=0.0,
        unconnected_amount=0.0,
        unsafe_amount=1.0,
        dark_amount=1.0,
    )
    generator.generate_connected_grid_map()

    assert generator.graph.number_of_edges() > 0
    for _, _, edge_data in generator.graph.edges(data=True):
        assert edge_data["dark"] is True
        assert edge_data["unsafe"] is True


def test_discretize_graph_splits_long_edges_into_shorter_segments() -> None:
    generator = DummyMapGenerator()
    generator.graph.add_node(0, pos=(0.0, 0.0))
    generator.graph.add_node(1, pos=(1.0, 0.0))
    generator.graph.add_node(2, pos=(4.0, 0.0))
    generator.graph.add_edge(0, 1, weight=1.0, dark=False, unsafe=False)
    generator.graph.add_edge(1, 2, weight=3.0, dark=False, unsafe=False)

    generator.discretize_graph()

    assert not generator.graph.has_edge(1, 2)
    assert generator.graph.number_of_nodes() == 5

    edge_lengths = [
        np.linalg.norm(
            np.array(generator.graph.nodes[u]["pos"])
            - np.array(generator.graph.nodes[v]["pos"])
        )
        for u, v in generator.graph.edges
    ]
    assert max(edge_lengths) <= 1.0 + 1e-9
