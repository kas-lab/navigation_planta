import networkx as nx
from .graph_generation import GraphGenerationMixin
from .map_io import MapIOPlottingMixin
from .pddl_builder import PDDLBuilderMixin


class MapGenerator(GraphGenerationMixin, MapIOPlottingMixin, PDDLBuilderMixin):
    def __init__(
            self,
            num_nodes=30,
            nodes_skip=0.1,
            unconnected_amount=0.15,
            unsafe_amount=0.25,
            dark_amount=0.25):
        self.graph = nx.Graph()
        self.num_nodes = num_nodes
        self.nodes_skip = nodes_skip
        self.unconnected_amount = unconnected_amount
        self.unsafe_amount = unsafe_amount
        self.dark_amount = dark_amount
