"""Shared helpers for no-adaptation navigation baselines."""

import unified_planning
from unified_planning.shortcuts import Object, Problem, BoolType

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


class MissionNoAdaptationMapGenerator(NoAdaptationMapGenerator):
    """No-adaptation baseline for mission-action experiments."""

    ACTION_NAMES = ['inspection', 'delivery', 'recharge', 'report']

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount, m):
        super().__init__(
            num_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
        assert 1 <= m <= 5, f'M must be in 1..5, got {m}'
        self.m = m
        self.active_actions = self.ACTION_NAMES[:m - 1]

    def generate_domain(self):
        super().generate_domain()
        for action_name in self.active_actions:
            setattr(
                self,
                f'{action_name}_waypoint_fluent',
                unified_planning.model.Fluent(
                    f'{action_name}_waypoint',
                    BoolType(),
                    wp=self.waypoint_type,
                ),
            )
            setattr(
                self,
                f'{action_name}_done_fluent',
                unified_planning.model.Fluent(
                    f'{action_name}_done',
                    BoolType(),
                ),
            )

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)

        if not self.active_actions:
            return

        sorted_nodes = sorted(self.graph.nodes)
        n = len(sorted_nodes)
        waypoints = {
            node_id: Object(f'wp{node_id}', self.waypoint_type)
            for node_id in self.graph.nodes
        }

        for i, action_name in enumerate(self.active_actions):
            wp_fluent = getattr(self, f'{action_name}_waypoint_fluent')
            done_fluent = getattr(self, f'{action_name}_done_fluent')
            self.problem.add_fluent(done_fluent, default_initial_value=False)

            target_node = sorted_nodes[(i + 1) * n // self.m]
            self.problem.set_initial_value(
                wp_fluent(waypoints[target_node]),
                True,
            )
            self.problem.add_goal(done_fluent())
