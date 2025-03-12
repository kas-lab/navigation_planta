import json
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D  # To create legend handles
import random
from scipy.spatial import Delaunay

from unified_planning.shortcuts import *
from unified_planning.io import PDDLWriter
from unified_planning.engines import PlanGenerationResultStatus

class MapGenerator:
    def __init__(
     self, 
     num_nodes = 30, 
     nodes_skip = 0.1, 
     unconnected_amount = 0.15, 
     unsafe_amount = 0.25, 
     dark_amount = 0.25):
        self.graph = nx.Graph()
        self.num_nodes = num_nodes
        self.nodes_skip = nodes_skip
        self.unconnected_amount = unconnected_amount
        self.unsafe_amount = unsafe_amount
        self.dark_amount = dark_amount

    def generate_graph(self):
        # Generate random office-like positions
        positions = np.random.rand(self.num_nodes, 2)  # (x, y) in [0,1]²

        # Delaunay triangulation for structured connectivity
        tri = Delaunay(positions)
        edges = set()
        for simplex in tri.simplices:
            for i in range(3):
                edges.add(tuple(sorted([simplex[i], simplex[(i+1)%3]])))

        # Construct graph with weighted edges
        for i, pos in enumerate(positions):
            self.graph.add_node(i, pos=pos)

        for u, v in edges:
            weight = np.linalg.norm(positions[u] - positions[v])  # Euclidean distance
            self.graph.add_edge(u, v, weight=weight, dark=False, unsafe=False)

        # Retain cycles by keeping extra edges from Delaunay
        # extra_edges = list(edges)
        # np.random.shuffle(extra_edges)
        # self.graph.add_edges_from(extra_edges[:len(extra_edges) // 3])  # Adding back some cycles

    def generate_grid_graph(self):
        # Define grid dimensions with some flexibility
        rows = int(np.ceil(np.sqrt(self.num_nodes)))  # Calculate rows
        cols = int(np.ceil(self.num_nodes / rows))  # Adjust columns based on rows

        # Generate grid-like positions with increasing distance between nodes
        positions = []
        for i in range(rows):
            for j in range(cols):
                if len(positions) < self.num_nodes:
                    # Node positions are spaced evenly
                    positions.append([j * 10, i * 10])  # Adjust '10' for larger distances

        positions = np.array(positions)

        # Randomly remove 10% of the nodes (skip 10% of the vertices)
        num_nodes_to_skip = int(self.num_nodes * self.nodes_skip)
        nodes_to_skip = random.sample(range(self.num_nodes), num_nodes_to_skip)
        positions = [pos for idx, pos in enumerate(positions) if idx not in nodes_to_skip]
        
        # Adjust the number of nodes after skipping
        self.num_node_resulting = self.num_nodes - num_nodes_to_skip

        # Construct graph with weighted edges
        graph_ = nx.Graph()
        for i, pos in enumerate(positions):
            graph_.add_node(i, pos=pos)

        # Add edges between adjacent nodes (either horizontal or vertical)
        edges = []
        for i in range(self.num_node_resulting):
            for j in range(i + 1, self.num_node_resulting):
                pos_i = positions[i]
                pos_j = positions[j]
                
                # Check if nodes are adjacent horizontally or vertically
                if (pos_i[0] == pos_j[0] and abs(pos_i[1] - pos_j[1]) == 10) or \
                (pos_i[1] == pos_j[1] and abs(pos_i[0] - pos_j[0]) == 10):
                    # Calculate Euclidean distance as edge weight
                    weight = np.linalg.norm(pos_i - pos_j)
                    graph_.add_edge(i, j, weight=weight, dark=False, unsafe=False)
                    edges.append((i, j))

        # Remove 10% of the edges
        total_edges = len(edges)
        num_edges_to_remove = int(self.unconnected_amount * total_edges)
        random.shuffle(edges)

        # Remove edges
        edges_to_remove = edges[:num_edges_to_remove]
        for u, v in edges_to_remove:
            graph_.remove_edge(u, v)
        
        return graph_

    def assign_dark_unsafe_corridors(self, graph):
        # Randomly assign "unsafe" and "dark" labels to 10% of the remaining edges
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
        graph = self.generate_grid_graph()
        # If not connected, generate a new graph
        while not nx.is_connected(graph):
            graph = self.generate_grid_graph()
        self.graph = self.assign_dark_unsafe_corridors(graph)


    def find_shortest_path(self, from_node=0, to_node=0):
        # Find shortest path using Dijkstra (example: node 0 to node 10)
        return nx.shortest_path(self.graph, source=from_node, target=to_node, weight='weight')

    def plot_graph(self, show_plot=False, save_file=False, filename="map.png"):
        fig, ax = plt.subplots(figsize=(8, 8))  # Create a figure and axis
        pos_ = {n: pos['pos'] for n, pos in self.graph.nodes().items()}
        nx.draw(self.graph, pos=pos_, with_labels=True, node_size=100, ax=ax)

        dark_corridors = [(u, v) for u, v in self.graph.edges if self.graph[u][v]['dark'] and not self.graph[u][v]['unsafe']]
        unsafe_corridors = [(u, v) for u, v in self.graph.edges if not self.graph[u][v]['dark'] and self.graph[u][v]['unsafe']]
        unsafe_dark_corridors = [(u, v) for u, v in self.graph.edges if self.graph[u][v]['dark'] and self.graph[u][v]['unsafe']]

        nx.draw_networkx_edges(self.graph, pos=pos_, edgelist=dark_corridors, edge_color='y', width=2, ax=ax)
        nx.draw_networkx_edges(self.graph, pos=pos_, edgelist=unsafe_corridors, edge_color='b', width=2, ax=ax)
        nx.draw_networkx_edges(self.graph, pos=pos_, edgelist=unsafe_dark_corridors, edge_color='r', width=2, ax=ax)

        # Create custom legend handles
        dark_handle = Line2D([0], [0], color='y', lw=2, label='Dark Corridors')
        unsafe_handle = Line2D([0], [0], color='b', lw=2, label='Unsafe Corridors')
        unsafe_dark_handle = Line2D([0], [0], color='r', lw=2, label='Unsafe and Dark Corridors')

        # Add legend to the plot
        ax.legend(handles=[dark_handle, unsafe_handle, unsafe_dark_handle], loc='best')

        if save_file:
            plt.savefig(filename, format='png', dpi=300, bbox_inches='tight')

        if show_plot:
            plt.show(block=True)  # Block only this figure
            plt.close(fig)  # Close the figure to avoid affecting other plots
        else:
            plt.close(fig)  # Ensure the figure is always closed if not shown


    def generate_domain_problem_files(self, save_domain=False, domain_filename='domain.pddl', save_problem=True, problem_filename='problem.pddl', init_goal=None):
        self.generate_domain()
        self.generate_problem(init_goal==None)
        if init_goal is not None:
            self.define_init_goal_waypoints(init_goal)
        writer = PDDLWriter(self.problem)
        if save_domain is True:
            writer.write_domain(domain_filename)
        if save_problem is True:
            writer.write_problem(problem_filename)
        

    def generate_domain(self):
        self.waypoint_type = UserType('waypoint')
        self.numerical_object_type = UserType('numerical-object')
        self.robot_at = unified_planning.model.Fluent(
            'robot_at', BoolType(), wp=self.waypoint_type)
        self.corridor = unified_planning.model.Fluent(
            'corridor', BoolType(), wp1=self.waypoint_type, wp2=self.waypoint_type)
        self.dark_corridor = unified_planning.model.Fluent(
            'dark_corridor', BoolType(), wp1=self.waypoint_type, wp2=self.waypoint_type)
        self.unsafe_corridor = unified_planning.model.Fluent(
            'unsafe_corridor', BoolType(), wp1=self.waypoint_type, wp2=self.waypoint_type)
        
        # self.distance = unified_planning.model.Fluent(
        #     'distance', BoolType(), wp1=self.waypoint_type, wp2=self.waypoint_type, d=self.numerical_object_type)
        
        self.move = InstantaneousAction('move', wp1=self.waypoint_type, wp2=self.waypoint_type)
        wp1 = self.move.parameter('wp1')
        wp2 = self.move.parameter('wp2')
        self.move.add_precondition(self.corridor(wp1, wp2))
        self.move.add_precondition(self.robot_at(wp1))
        self.move.add_effect(self.robot_at(wp1), False) 
        self.move.add_effect(self.robot_at(wp2), True)

    def generate_problem(self, add_init_goal = True):
        self.problem = Problem(name = 'navigation')
        self.problem.add_fluent(self.robot_at, default_initial_value=False)
        self.problem.add_fluent(self.corridor, default_initial_value=False)
        self.problem.add_action(self.move)

        waypoints = {node_id: Object('wp%s' % node_id, self.waypoint_type) for node_id in self.graph.nodes}
        self.problem.add_objects(waypoints.values())
        for u, v in self.graph.edges:
            self.problem.set_initial_value(self.corridor(waypoints[u], waypoints[v]), True)
            self.problem.set_initial_value(self.corridor(waypoints[v], waypoints[u]), True)
            if 'dark' in self.graph.edges[u, v] and self.graph.edges[u, v]['dark'] == True:
                self.problem.set_initial_value(self.dark_corridor(waypoints[u], waypoints[v]), True)
                self.problem.set_initial_value(self.dark_corridor(waypoints[v], waypoints[u]), True)
            if 'unsafe' in self.graph.edges[u, v] and self.graph.edges[u, v]['unsafe'] == True:
                self.problem.set_initial_value(self.unsafe_corridor(waypoints[u], waypoints[v]), True)
                self.problem.set_initial_value(self.unsafe_corridor(waypoints[v], waypoints[u]), True)

        if add_init_goal is True:
            sorted_waypoints = sorted(waypoints.keys())
            init_wp_key= sorted_waypoints[0]
            goal_wp_key= sorted_waypoints[-1]
            self.problem.set_initial_value(self.robot_at(waypoints[init_wp_key]), True)
            self.problem.add_goal(self.robot_at(waypoints[goal_wp_key]))

    def define_init_goal_waypoints(self, init_goal):
        self.problem.set_initial_value(self.robot_at(Object('wp%i' % init_goal[0], self.waypoint_type)), True)
        self.problem.add_goal(self.robot_at(Object('wp%i' % init_goal[1], self.waypoint_type)))

    def solve_plan(self):
        planner = OneshotPlanner(name="fast-downward")
        result = planner.solve(self.problem)
        if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
            print(f'Found a plan.\nThe plan is: {result.plan}')
        else:
            print("No plan found.")
    
    def load_json(self, file_path):
        with open(file_path, "r") as file:
            data = json.load(file)

        # Create a graph
        self.graph = nx.Graph()

        # Extract nodes and positions from the "map" section
        positions = {}
        for node in data["map"]:
            node_id = int(node["node-id"][1:])
            x, y = node["coords"]["x"], node["coords"]["y"]
            positions[node_id] = (x, y)
            self.graph.add_node(node_id, pos=(x, y))

        # Extract edges from the "map" section
        for node in data["map"]:
            node_id = int(node["node-id"][1:])
            for neighbor in node["connected-to"]:
                self.graph.add_edge(node_id, int(neighbor[1:]), dark=False, unsafe=False)
        # l15 to l14 - unsafe
        self.graph[15][14]['unsafe'] = True
        # l18 to l19 - dark
        self.graph[18][19]['dark'] = True
        # l3  to l4  - dark
        self.graph[3][4]['dark'] = True
        # l6  to l8  - unsafe
        self.graph[6][8]['unsafe'] = True
        # l56 to l55 - unsafe
        self.graph[56][55]['unsafe'] = True
        # l12 to l11 - dark and unsafe
        self.graph[12][11]['unsafe'] = True
        self.graph[12][11]['dark'] = True

if __name__ == '__main__':
    mp = MapGenerator()
    mp.generate_connected_grid_map()
    mp.generate_domain_problem_files()
    mp.solve_plan()
    mp.plot_graph(show_plot=True)