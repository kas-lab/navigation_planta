"""PDDL domain/problem construction helpers for navigation scenarios."""

import re

import unified_planning
from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.io import PDDLWriter
from unified_planning.io.pddl_writer import ConverterToPDDLString
from unified_planning.shortcuts import (
    BoolType,
    InstantaneousAction,
    Object,
    OneshotPlanner,
    Problem,
    UserType,
)


class PDDLBuilderMixin:
    def generate_domain_problem_files(
            self,
            save_domain=False,
            domain_filename='domain.pddl',
            save_problem=True,
            problem_filename='problem.pddl',
            init_goal=None):
        self.generate_domain()
        self.generate_problem(init_goal is None)
        if init_goal is not None:
            self.define_init_goal_waypoints(init_goal)
        writer = PDDLWriter(self.problem)
        if save_domain is True:
            writer.write_domain(domain_filename)
        if save_problem is True:
            self.write_problem_fast(problem_filename)

        with open(problem_filename, "r") as file:
            content = file.read()
            updated_content = re.sub(r"o_0_0_decimal", "0.0_decimal", content)
            updated_content = re.sub(
                r"o_0_8_decimal", "0.8_decimal", updated_content)
            updated_content = re.sub(
                r"o_1_0_decimal", "1.0_decimal", updated_content)

        with open(problem_filename, "w") as file:
            file.write(updated_content)

    def write_problem_fast(self, problem_filename):
        writer = PDDLWriter(self.problem)
        converter = ConverterToPDDLString(
            self.problem.environment,
            writer._get_mangled_name)
        problem_name = self.problem.name or "pddl"

        with open(problem_filename, "w") as out:
            out.write(f"(define (problem {problem_name}-problem)\n")
            out.write(f" (:domain {problem_name}-domain)\n")
            if len(self.problem.user_types) > 0:
                out.write(" (:objects")
                for user_type in self.problem.user_types:
                    objects = [
                        obj for obj in self.problem.all_objects
                        if obj.type == user_type]
                    if len(objects) > 0:
                        out.write(
                            f'\n   {" ".join(writer._get_mangled_name(obj) for obj in objects)} - {writer._get_mangled_name(user_type)}')
                out.write("\n )\n")

            out.write(" (:init")
            explicit_initial_values = sorted(
                self.problem.explicit_initial_values.items(),
                key=lambda item: converter.convert(item[0]))
            for fluent, value in explicit_initial_values:
                if value.is_true():
                    out.write("\n              ")
                    out.write(f" {converter.convert(fluent)}")
                elif not value.is_false():
                    out.write("\n              ")
                    out.write(
                        f" (= {converter.convert(fluent)} {converter.convert(value)})")
            out.write("\n )\n")

            goals_str = []
            for goal in (condition.simplify() for condition in self.problem.goals):
                if goal.is_and():
                    goals_str.extend(map(converter.convert, goal.args))
                else:
                    goals_str.append(converter.convert(goal))
            goals_string = "\n           ".join(goals_str)
            out.write(f" (:goal (and \n           {goals_string}\n        )\n )\n")
            out.write(")\n")

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
        self.light_requirement = unified_planning.model.Fluent(
            'light_requirement',
            BoolType(),
            wp1=self.waypoint_type,
            wp2=self.waypoint_type,
            v=self.numerical_object_type)
        self.safety_requirement = unified_planning.model.Fluent(
            'safety_requirement',
            BoolType(),
            wp1=self.waypoint_type,
            wp2=self.waypoint_type,
            v=self.numerical_object_type)

        self.move = InstantaneousAction(
            'move', wp1=self.waypoint_type, wp2=self.waypoint_type)
        wp1 = self.move.parameter('wp1')
        wp2 = self.move.parameter('wp2')
        self.move.add_precondition(self.corridor(wp1, wp2))
        self.move.add_precondition(self.robot_at(wp1))
        self.move.add_effect(self.robot_at(wp1), False)
        self.move.add_effect(self.robot_at(wp2), True)

    def generate_problem(self, add_init_goal=True):
        self.problem = Problem(name='navigation')
        self.problem.add_fluent(self.robot_at, default_initial_value=False)
        self.problem.add_fluent(self.corridor, default_initial_value=False)
        self.problem.add_action(self.move)

        waypoints = {
            node_id: Object(
                'wp%s' %
                node_id,
                self.waypoint_type) for node_id in self.graph.nodes}
        self.problem.add_objects(waypoints.values())
        zero_decimal = Object("0.0_decimal", self.numerical_object_type)
        zero_eight_decimal = Object("0.8_decimal", self.numerical_object_type)
        one_decimal = Object("1.0_decimal", self.numerical_object_type)
        for u, v in self.graph.edges:
            self.problem.set_initial_value(
                self.corridor(waypoints[u], waypoints[v]), True)
            self.problem.set_initial_value(
                self.corridor(waypoints[v], waypoints[u]), True)
            if 'dark' in self.graph.edges[u, v] and self.graph.edges[u, v]['dark']:
                self.problem.set_initial_value(self.light_requirement(
                    waypoints[u], waypoints[v], one_decimal), True)
                self.problem.set_initial_value(self.light_requirement(
                    waypoints[v], waypoints[u], one_decimal), True)
            else:
                self.problem.set_initial_value(self.light_requirement(
                    waypoints[u], waypoints[v], zero_decimal), True)
                self.problem.set_initial_value(self.light_requirement(
                    waypoints[v], waypoints[u], zero_decimal), True)
            if 'unsafe' in self.graph.edges[u, v] and self.graph.edges[u, v]['unsafe']:
                self.problem.set_initial_value(self.safety_requirement(
                    waypoints[u], waypoints[v], zero_eight_decimal), True)
                self.problem.set_initial_value(self.safety_requirement(
                    waypoints[v], waypoints[u], zero_eight_decimal), True)
            else:
                self.problem.set_initial_value(self.safety_requirement(
                    waypoints[u], waypoints[v], zero_decimal), True)
                self.problem.set_initial_value(self.safety_requirement(
                    waypoints[v], waypoints[u], zero_decimal), True)

        if add_init_goal is True:
            sorted_waypoints = sorted(waypoints.keys())
            init_wp_key = sorted_waypoints[0]
            goal_wp_key = sorted_waypoints[-1]
            self.problem.set_initial_value(
                self.robot_at(waypoints[init_wp_key]), True)
            self.problem.add_goal(self.robot_at(waypoints[goal_wp_key]))

    def define_init_goal_waypoints(self, init_goal):
        self.problem.set_initial_value(
            self.robot_at(
                Object(
                    'wp%i' %
                    init_goal[0],
                    self.waypoint_type)),
            True)
        self.problem.add_goal(
            self.robot_at(
                Object(
                    'wp%i' %
                    init_goal[1],
                    self.waypoint_type)))

    def solve_plan(self):
        planner = OneshotPlanner(name="fast-downward")
        result = planner.solve(self.problem)
        if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
            print(f'Found a plan.\nThe plan is: {result.plan}')
        else:
            print("No plan found.")
