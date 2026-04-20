"""Shared helpers for no-adaptation navigation baselines."""

from unified_planning.shortcuts import Object, Problem

from .map_generator import MapGenerator


class NoAdaptationMapGenerator(MapGenerator):
    """Generate a problem compatible with the no-adaptation baseline domain."""

    def generate_problem(self, add_init_goal=True):
        self.problem = Problem(name='navigation')
        self.problem.add_fluent(self.robot_at, default_initial_value=False)
        self.problem.add_fluent(self.corridor, default_initial_value=False)
        self.problem.add_action(self.move)

        waypoints = {
            node_id: Object(f'wp{node_id}', self.waypoint_type)
            for node_id in self.graph.nodes
        }
        self.problem.add_objects(waypoints.values())

        for u, v in self.graph.edges:
            self.problem.set_initial_value(
                self.corridor(waypoints[u], waypoints[v]), True)
            self.problem.set_initial_value(
                self.corridor(waypoints[v], waypoints[u]), True)

        if add_init_goal:
            sorted_waypoints = sorted(waypoints.keys())
            init_wp_key = sorted_waypoints[0]
            goal_wp_key = sorted_waypoints[-1]
            self.problem.set_initial_value(
                self.robot_at(waypoints[init_wp_key]), True)
            self.problem.add_goal(self.robot_at(waypoints[goal_wp_key]))
