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
    def __init__(self):
        self.G = nx.Graph()
        self.num_nodes = 30
        self.nodes_skip = 0.1 # 10%
        self.unconnected_amount = 0.15 # 10%
        self.unsafe_amount = 0.25 # 10%
        self.dark_amount = 0.25 # 10%

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
            self.G.add_node(i, pos=pos)

        for u, v in edges:
            weight = np.linalg.norm(positions[u] - positions[v])  # Euclidean distance
            self.G.add_edge(u, v, weight=weight, dark=False, unsafe=False)

        # Retain cycles by keeping extra edges from Delaunay
        # extra_edges = list(edges)
        # np.random.shuffle(extra_edges)
        # self.G.add_edges_from(extra_edges[:len(extra_edges) // 3])  # Adding back some cycles

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
        self.num_nodes -= num_nodes_to_skip

        # Construct graph with weighted edges
        self.G = nx.Graph()
        for i, pos in enumerate(positions):
            self.G.add_node(i, pos=pos)

        # Add edges between adjacent nodes (either horizontal or vertical)
        edges = []
        for i in range(self.num_nodes):
            for j in range(i + 1, self.num_nodes):
                pos_i = positions[i]
                pos_j = positions[j]
                
                # Check if nodes are adjacent horizontally or vertically
                if (pos_i[0] == pos_j[0] and abs(pos_i[1] - pos_j[1]) == 10) or \
                (pos_i[1] == pos_j[1] and abs(pos_i[0] - pos_j[0]) == 10):
                    # Calculate Euclidean distance as edge weight
                    weight = np.linalg.norm(pos_i - pos_j)
                    self.G.add_edge(i, j, weight=weight, dark=False, unsafe=False)
                    edges.append((i, j))

        # Remove 10% of the edges but ensure connectivity
        total_edges = len(edges)
        num_edges_to_remove = int(self.unconnected_amount * total_edges)
        random.shuffle(edges)

        # Temporarily remove edges and check for connectivity
        edges_to_remove = edges[:num_edges_to_remove]
        for u, v in edges_to_remove:
            self.G.remove_edge(u, v)

        # Ensure the graph remains connected
        if not nx.is_connected(self.G):
            weight = np.linalg.norm(edges[0][0] - edges[0][1])
            self.G.add_edge(edges[0][0], edges[0][1], weight=weight, dark=False, unsafe=False)  # Reconnect first edge if disconnected

        # Randomly assign "unsafe" and "dark" labels to 10% of the remaining edges
        remaining_edges = list(self.G.edges)
        num_unsafe_edges = int(self.unsafe_amount * len(remaining_edges))
        num_dark_edges = int(self.dark_amount * len(remaining_edges))

        # Assign "unsafe" to 10% of edges
        unsafe_edges = random.sample(remaining_edges, num_unsafe_edges)
        for u, v in unsafe_edges:
            self.G[u][v]['unsafe'] = True

        # Assign "dark" to 10% of edges
        dark_edges = random.sample(remaining_edges, num_dark_edges)
        for u, v in dark_edges:
            self.G[u][v]['dark'] = True


    def find_shortest_path(self, from_node=0, to_node=0):
        # Find shortest path using Dijkstra (example: node 0 to node 10)
        return nx.shortest_path(self.G, source=from_node, target=to_node, weight='weight')

    def plot_graph(self):
        plt.figure(figsize=(8, 8))
        pos_ = {n: pos['pos'] for n, pos in self.G.nodes().items()}
        nx.draw(self.G, pos=pos_, with_labels=True, node_size=100)
        
        dark_corridors = [(u, v) for u,v in self.G.edges if self.G[u][v]['dark'] == True and self.G[u][v]['unsafe'] == False]
        unsafe_corridors = [(u, v) for u,v in self.G.edges if self.G[u][v]['dark'] == False and self.G[u][v]['unsafe'] == True]
        unsafe_dark_corridors = [(u, v) for u,v in self.G.edges if self.G[u][v]['dark'] == True and self.G[u][v]['unsafe'] == True]
        
        nx.draw_networkx_edges(self.G, pos=pos_, edgelist=dark_corridors, edge_color='y', width=2)
        nx.draw_networkx_edges(self.G, pos=pos_, edgelist=unsafe_corridors, edge_color='b', width=2)
        nx.draw_networkx_edges(self.G, pos=pos_, edgelist=unsafe_dark_corridors, edge_color='r', width=2)

        nx.draw_networkx_edges(self.G, pos=pos_)

        # Create custom legend handles
        dark_handle = Line2D([0], [0], color='y', lw=2, label='Dark Corridors')
        unsafe_handle = Line2D([0], [0], color='b', lw=2, label='Unsafe Corridors')
        unsafe_dark_handle = Line2D([0], [0], color='r', lw=2, label='Unsafe and Dark Corridors')

        # Add legend to the plot
        plt.legend(handles=[dark_handle, unsafe_handle, unsafe_dark_handle], loc='best')
        plt.show()

    def generate_domain_problem_files(self):
        self.generate_domain()
        self.generate_problem()
        self.write_pddl_files()
        

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

    def generate_problem(self):
        self.problem = Problem('navigation')
        self.problem.add_fluent(self.robot_at, default_initial_value=False)
        self.problem.add_fluent(self.corridor, default_initial_value=False)
        self.problem.add_action(self.move)

        waypoints = [Object('wp%s' % i, self.waypoint_type) for i in range(self.num_nodes)]
        self.problem.add_objects(waypoints)
        for u, v in self.G.edges:
            self.problem.set_initial_value(self.corridor(waypoints[u], waypoints[v]), True)
            if 'dark' in self.G.edges[u, v] and self.G.edges[u, v]['dark'] == True:
                self.problem.set_initial_value(self.dark_corridor(waypoints[u], waypoints[v]), True)
            if 'unsafe' in self.G.edges[u, v] and self.G.edges[u, v]['unsafe'] == True:
                self.problem.set_initial_value(self.unsafe_corridor(waypoints[u], waypoints[v]), True)

        self.problem.set_initial_value(self.robot_at(waypoints[0]), True)
        self.problem.add_goal(self.robot_at(waypoints[-1]))

    def solve_plan(self):
        planner = OneshotPlanner(name="fast-downward")
        result = planner.solve(self.problem)
        if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
            print(f'Found a plan.\nThe plan is: {result.plan}')
        else:
            print("No plan found.")
    
    def write_pddl_files(self):
        writer = PDDLWriter(self.problem)
        # domain_filename = "domain.pddl"
        # writer.write_domain(domain_filename)
        problem_filename = "pddl/problem.pddl"
        writer.write_problem(problem_filename)

if __name__ == '__main__':
    mp = MapGenerator()
    # mp.generate_graph()
    mp.generate_grid_graph()
    mp.generate_domain_problem_files()
    mp.solve_plan()
    mp.plot_graph()