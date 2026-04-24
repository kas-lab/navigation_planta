"""Graph generation and pathfinding helpers for navigation scenarios."""

import random

import networkx as nx
import numpy as np
from scipy.spatial import Delaunay


class GraphGenerationMixin:
    # def generate_graph(self):
    #     # Generate random office-like positions
    #     positions = np.random.rand(self.num_nodes, 2)  # (x, y) in [0,1]²

    #     # Delaunay triangulation for structured connectivity
    #     tri = Delaunay(positions)
    #     edges = set()
    #     for simplex in tri.simplices:
    #         for i in range(3):
    #             edges.add(tuple(sorted([simplex[i], simplex[(i + 1) % 3]])))

    #     # Construct graph with weighted edges
    #     for i, pos in enumerate(positions):
    #         self.graph.add_node(i, pos=pos)

    #     for u, v in edges:
    #         weight = np.linalg.norm(
    #             positions[u] - positions[v])  # Euclidean distance
    #         self.graph.add_edge(u, v, weight=weight, dark=False, unsafe=False)

    def generate_grid_graph(self):
        rows = int(np.ceil(np.sqrt(self.num_nodes)))
        cols = int(np.ceil(self.num_nodes / rows))

        cells = []
        for row in range(rows):
            for col in range(cols):
                if len(cells) < self.num_nodes:
                    cells.append((row, col))

        target_nodes = max(1, self.num_nodes - int(self.num_nodes * self.nodes_skip))
        self.num_node_resulting = target_nodes
        valid_cells = set(cells)

        def neighbors(cell):
            row, col = cell
            candidate_neighbors = (
                (row - 1, col),
                (row + 1, col),
                (row, col - 1),
                (row, col + 1),
            )
            return [neighbor for neighbor in candidate_neighbors if neighbor in valid_cells]

        start_cell = random.choice(cells)
        kept_cells = {start_cell}
        frontier = set(neighbors(start_cell))

        while len(kept_cells) < target_nodes:
            next_cell = random.choice(tuple(frontier))
            frontier.remove(next_cell)
            kept_cells.add(next_cell)
            for neighbor in neighbors(next_cell):
                if neighbor not in kept_cells:
                    frontier.add(neighbor)

        ordered_cells = sorted(kept_cells)
        cell_to_node_id = {cell: node_id for node_id, cell in enumerate(ordered_cells)}

        graph_ = nx.Graph()
        for node_id, (row, col) in enumerate(ordered_cells):
            graph_.add_node(node_id, pos=np.array([col * 10, row * 10]))

        for cell in ordered_cells:
            for neighbor in neighbors(cell):
                if neighbor in kept_cells and cell < neighbor:
                    graph_.add_edge(
                        cell_to_node_id[cell],
                        cell_to_node_id[neighbor],
                        weight=10.0,
                        dark=False,
                        unsafe=False)

        spanning_tree = nx.dfs_tree(
            graph_,
            source=random.choice(list(graph_.nodes))).to_undirected()
        removable_edges = [
            edge for edge in graph_.edges if not spanning_tree.has_edge(*edge)]
        num_edges_to_remove = min(
            int(self.unconnected_amount * graph_.number_of_edges()),
            len(removable_edges))
        random.shuffle(removable_edges)

        for u, v in removable_edges[:num_edges_to_remove]:
            graph_.remove_edge(u, v)

        return graph_

    def assign_dark_unsafe_corridors(self, graph):
        # Randomly assign "unsafe" and "dark" labels to 10% of the remaining
        # edges
        remaining_edges = list(graph.edges)
        num_unsafe_edges = int(self.unsafe_amount * len(remaining_edges))
        num_dark_edges = int(self.dark_amount * len(remaining_edges))

        # Assign "unsafe" to 10% of edges
        unsafe_edges = random.sample(remaining_edges, num_unsafe_edges)
        for u, v in unsafe_edges:
            graph[u][v]['unsafe'] = True

        # Assign "dark" to 10% of edges
        dark_edges = random.sample(remaining_edges, num_dark_edges)
        for u, v in dark_edges:
            graph[u][v]['dark'] = True

        return graph

    def generate_connected_grid_map(self):
        self.graph = self.assign_dark_unsafe_corridors(self.generate_grid_graph())

    def find_shortest_path(self, from_node=0, to_node=0):
        # Find shortest path using Dijkstra (example: node 0 to node 10)
        return nx.shortest_path(
            self.graph,
            source=from_node,
            target=to_node,
            weight='weight')
