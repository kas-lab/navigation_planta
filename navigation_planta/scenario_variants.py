"""Package-level scenario-specific MapGenerator variants."""

import random

import unified_planning
from unified_planning.shortcuts import BoolType, Object

from .map_generator import MapGenerator
from .no_adaptation import NoAdaptationMapGenerator


def _build_waypoints(generator) -> dict[int, Object]:
    return {
        node_id: Object(f'wp{node_id}', generator.waypoint_type)
        for node_id in generator.graph.nodes
    }


def _build_decimal_objects(generator) -> tuple[Object, Object]:
    return (
        Object('0.0_decimal', generator.numerical_object_type),
        Object('0.8_decimal', generator.numerical_object_type),
    )


def _initialize_edge_flags(graph, *flag_names: str) -> None:
    for u, v in graph.edges:
        for flag_name in flag_names:
            graph[u][v][flag_name] = False


def _mark_edge_fraction(graph, *, flag_name: str, amount: float) -> None:
    if amount <= 0:
        return

    remaining_edges = list(graph.edges)
    if not remaining_edges:
        return

    count = min(int(amount * len(remaining_edges)), len(remaining_edges))
    for u, v in random.sample(remaining_edges, count):
        graph[u][v][flag_name] = True


class CorridorTypeFeatureMixin:
    """Reusable corridor-type feature composition."""

    def _configure_corridor_type_features(
            self,
            *,
            outdoor_amount: float = 0.0,
            dusty_amount: float = 0.0) -> None:
        self.outdoor_amount = outdoor_amount
        self.dusty_amount = dusty_amount

    def assign_dark_unsafe_corridors(self, graph):
        _initialize_edge_flags(graph, 'outdoor', 'dusty')
        graph = super().assign_dark_unsafe_corridors(graph)
        _mark_edge_fraction(
            graph,
            flag_name='outdoor',
            amount=self.outdoor_amount,
        )
        _mark_edge_fraction(
            graph,
            flag_name='dusty',
            amount=self.dusty_amount,
        )
        return graph

    def _add_corridor_type_fluents(self) -> None:
        if self.outdoor_amount > 0:
            self.outdoor_requirement = unified_planning.model.Fluent(
                'outdoor_requirement', BoolType(),
                wp1=self.waypoint_type,
                wp2=self.waypoint_type,
                v=self.numerical_object_type,
            )
        if self.dusty_amount > 0:
            self.dust_requirement = unified_planning.model.Fluent(
                'dust_requirement', BoolType(),
                wp1=self.waypoint_type,
                wp2=self.waypoint_type,
                v=self.numerical_object_type,
            )

    def _populate_corridor_type_requirements(self) -> None:
        if self.outdoor_amount <= 0 and self.dusty_amount <= 0:
            return

        waypoints = _build_waypoints(self)
        zero_decimal, zero_eight_decimal = _build_decimal_objects(self)

        for u, v in self.graph.edges:
            if self.outdoor_amount > 0:
                req = zero_eight_decimal if self.graph[u][v].get('outdoor') else zero_decimal
                self.problem.set_initial_value(
                    self.outdoor_requirement(waypoints[u], waypoints[v], req), True)
                self.problem.set_initial_value(
                    self.outdoor_requirement(waypoints[v], waypoints[u], req), True)
            if self.dusty_amount > 0:
                req = zero_eight_decimal if self.graph[u][v].get('dusty') else zero_decimal
                self.problem.set_initial_value(
                    self.dust_requirement(waypoints[u], waypoints[v], req), True)
                self.problem.set_initial_value(
                    self.dust_requirement(waypoints[v], waypoints[u], req), True)


class MissionActionFeatureMixin:
    """Reusable mission-action feature composition."""

    ACTION_NAMES = ['inspection', 'delivery', 'recharge', 'report']

    def _configure_mission_action_features(self, *, m: int) -> None:
        assert 1 <= m <= 5, f'M must be in 1..5, got {m}'
        self.m = m
        self.active_actions = self.ACTION_NAMES[:m - 1]

    def _add_mission_action_fluents(self) -> None:
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

    def _populate_mission_action_goals(self) -> None:
        if not self.active_actions:
            return

        sorted_nodes = sorted(self.graph.nodes)
        n = len(sorted_nodes)
        waypoints = _build_waypoints(self)

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


class CorridorTypeMapGenerator(CorridorTypeFeatureMixin, MapGenerator):
    """MapGenerator extended with extra corridor-type requirements."""

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount,
                 outdoor_amount=0.0, dusty_amount=0.0):
        super().__init__(
            num_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
        self._configure_corridor_type_features(
            outdoor_amount=outdoor_amount,
            dusty_amount=dusty_amount,
        )

    def generate_domain(self):
        super().generate_domain()
        self._add_corridor_type_fluents()

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)
        self._populate_corridor_type_requirements()


class MissionActionMapGenerator(MissionActionFeatureMixin, MapGenerator):
    """MapGenerator extended with secondary mission action types."""

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount, m):
        super().__init__(
            num_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
        self._configure_mission_action_features(m=m)

    def generate_domain(self):
        super().generate_domain()
        self._add_mission_action_fluents()

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)
        self._populate_mission_action_goals()


class MissionActionNoAdaptationMapGenerator(
        MissionActionFeatureMixin,
        NoAdaptationMapGenerator):
    """No-adaptation baseline for mission-action experiments."""

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount, m):
        super().__init__(
            num_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
        self._configure_mission_action_features(m=m)

    def generate_domain(self):
        super().generate_domain()
        self._add_mission_action_fluents()

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)
        self._populate_mission_action_goals()


class CombinedScenarioMapGenerator(
        CorridorTypeFeatureMixin,
        MissionActionFeatureMixin,
        MapGenerator):
    """MapGenerator at maximum adaptive complexity: K=4 and M=5."""

    def __init__(self, num_nodes, nodes_skip, unconnected_amount,
                 unsafe_amount, dark_amount,
                 outdoor_amount=0.20, dusty_amount=0.20):
        super().__init__(
            num_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
        self._configure_corridor_type_features(
            outdoor_amount=outdoor_amount,
            dusty_amount=dusty_amount,
        )
        self._configure_mission_action_features(m=5)

    def generate_domain(self):
        super().generate_domain()
        self._add_corridor_type_fluents()
        self._add_mission_action_fluents()

    def generate_problem(self, add_init_goal=True):
        super().generate_problem(add_init_goal)
        self._populate_corridor_type_requirements()
        self._populate_mission_action_goals()
