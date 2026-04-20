"""Map loading, plotting, and discretization helpers."""

import json
import math

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import networkx as nx
import numpy as np


class MapIOPlottingMixin:
    def plot_graph(self, show_plot=False, save_file=False, filename="map.png", legend=True, node_labels=True):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.axis("off")
        pos_ = {n: pos['pos'] for n, pos in self.graph.nodes().items()}

        nodes = list(self.graph.nodes)
        first_node = nodes[0]
        last_node = nodes[-1]

        normal_nodes = [n for n in self.graph.nodes if n != first_node and n != last_node]
        nx.draw_networkx_nodes(
            self.graph,
            pos=pos_,
            nodelist=normal_nodes,
            node_color='lightgray',
            node_size=100,
            ax=ax)

        nx.draw_networkx_nodes(
            self.graph,
            pos=pos_,
            nodelist=[first_node],
            node_color='#377eb8',
            node_size=150,
            ax=ax,
            label="Initial location")

        nx.draw_networkx_nodes(
            self.graph,
            pos=pos_,
            nodelist=[last_node],
            node_color='#e41a1c',
            node_size=150,
            ax=ax,
            label="Goal location")

        nx.draw_networkx_labels(
            self.graph,
            pos=pos_,
            labels={first_node: "I", last_node: "G"},
            font_size=10,
            ax=ax)

        dark_corridors = [(u, v) for u, v in self.graph.edges if self.graph[u]
                          [v]['dark'] and not self.graph[u][v]['unsafe']]
        unsafe_corridors = [
            (u,
             v) for u,
            v in self.graph.edges if not self.graph[u][v]['dark'] and self.graph[u][v]['unsafe']]
        unsafe_dark_corridors = [
            (u,
             v) for u,
            v in self.graph.edges if self.graph[u][v]['dark'] and self.graph[u][v]['unsafe']]

        normal_corridors = [
            (u,
             v) for u,
            v in self.graph.edges if not self.graph[u][v]['dark'] and not self.graph[u][v]['unsafe']]

        nx.draw_networkx_edges(
            self.graph,
            pos=pos_,
            edgelist=normal_corridors,
            edge_color='black',
            width=1,
            ax=ax)

        nx.draw_networkx_edges(
            self.graph,
            pos=pos_,
            edgelist=dark_corridors,
            edge_color='y',
            width=2,
            ax=ax)
        nx.draw_networkx_edges(
            self.graph,
            pos=pos_,
            edgelist=unsafe_corridors,
            edge_color='b',
            width=2,
            ax=ax)
        nx.draw_networkx_edges(
            self.graph,
            pos=pos_,
            edgelist=unsafe_dark_corridors,
            edge_color='r',
            width=2,
            ax=ax)

        dark_handle = Line2D([0], [0], color='y', lw=2, label='Dark Corridors')
        unsafe_handle = Line2D(
            [0],
            [0],
            color='b',
            lw=2,
            label='Unsafe Corridors')
        unsafe_dark_handle = Line2D(
            [0], [0], color='r', lw=2, label='Unsafe and Dark Corridors')

        if legend:
            ax.legend(
                handles=[
                    dark_handle,
                    unsafe_handle,
                    unsafe_dark_handle],
                loc='best')

        if save_file:
            plt.savefig(filename, format='png', dpi=300, bbox_inches='tight', pad_inches=0)

        if show_plot:
            plt.show(block=True)
            plt.close(fig)
        else:
            plt.close(fig)

    def load_json(self, file_path):
        with open(file_path, "r") as file:
            data = json.load(file)

        self.graph = nx.Graph()

        positions = {}
        for node in data["map"]:
            node_id = int(node["node-id"][1:])
            x, y = node["coords"]["x"], node["coords"]["y"]
            positions[node_id] = (x, y)
            self.graph.add_node(node_id, pos=(x, y))

        for node in data["map"]:
            node_id = int(node["node-id"][1:])
            for neighbor in node["connected-to"]:
                weight = math.dist(
                    positions[node_id], positions[int(neighbor[1:])])
                self.graph.add_edge(node_id,
                                    int(neighbor[1:]),
                                    weight=weight,
                                    dark=False,
                                    unsafe=False)

        for hitrate in data["hitrate"]:
            to_node = int(hitrate["to"][1:])
            from_node = int(hitrate["from"][1:])
            if self.graph.has_edge(from_node, to_node):
                self.graph[from_node][to_node]["hitrate"] = hitrate
            if self.graph.has_edge(to_node, from_node):
                self.graph[to_node][from_node]["hitrate"] = hitrate

        self.graph[15][14]['unsafe'] = True
        self.graph[18][19]['dark'] = True
        self.graph[3][4]['dark'] = True
        self.graph[6][8]['unsafe'] = True
        self.graph[56][55]['unsafe'] = True
        self.graph[12][11]['unsafe'] = True
        self.graph[12][11]['dark'] = True

    def load_and_discretize_json(self, file_path):
        self.load_json(file_path)
        self.discretize_graph()

    def discretize_graph(self):
        """Discretizes the graph so that all edges have the same maximum possible length."""
        edge_lengths = [
            np.linalg.norm(
                np.array(
                    self.graph.nodes[u]['pos']) -
                np.array(
                    self.graph.nodes[v]['pos'])) for u,
            v in self.graph.edges]

        max_length = min(edge_lengths)

        new_edges = []
        new_nodes = {}
        max_node_id = max(self.graph.nodes)

        for u, v in list(self.graph.edges):
            pos_u = np.array(self.graph.nodes[u]['pos'])
            pos_v = np.array(self.graph.nodes[v]['pos'])
            dist = np.linalg.norm(pos_v - pos_u)

            if dist > max_length:
                num_segments = int(np.ceil(dist / max_length))
                prev_node = u

                for i in range(1, num_segments):
                    new_pos = pos_u + (pos_v - pos_u) * (i / num_segments)
                    max_node_id += 1
                    new_nodes[max_node_id] = tuple(new_pos)
                    new_edges.append((prev_node, max_node_id))
                    prev_node = max_node_id

                new_edges.append((prev_node, v))
                self.graph.remove_edge(u, v)

        for node_id, pos in new_nodes.items():
            self.graph.add_node(node_id, pos=pos)
        self.graph.add_edges_from(new_edges, dark=False, unsafe=False)
