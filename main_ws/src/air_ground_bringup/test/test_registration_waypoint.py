#!/usr/bin/env python3

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest

import numpy as np


MISSION_PATH = Path(__file__).parents[1] / "scripts" / "uav_sphere_mission.py"


class FakeDuration:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return FakeDuration(self.seconds - other.seconds)


class FakeRospy:
    class Time:
        @staticmethod
        def now():
            return FakeTime(100.0)

    @staticmethod
    def loginfo(*_args):
        pass


def load_pure_function(name):
    tree = ast.parse(MISSION_PATH.read_text(), filename=str(MISSION_PATH))
    function = next(
        (node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name),
        None,
    )
    if function is None:
        return None
    namespace = {"math": math, "rospy": FakeRospy}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(MISSION_PATH), "exec"), namespace)
    return namespace[name]


def load_mission_class():
    tree = ast.parse(MISSION_PATH.read_text(), filename=str(MISSION_PATH))
    definitions = [
        node
        for node in tree.body
        if (isinstance(node, ast.FunctionDef) and node.name == "registration_waypoint")
        or (isinstance(node, ast.ClassDef) and node.name == "Mission")
    ]
    namespace = {"math": math, "rospy": FakeRospy}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(MISSION_PATH), "exec"), namespace)
    return namespace["Mission"]


def execute_registration_parameter_assignments(get_param):
    tree = ast.parse(MISSION_PATH.read_text(), filename=str(MISSION_PATH))
    mission = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Mission")
    initializer = next(
        node for node in mission.body if isinstance(node, ast.FunctionDef) and node.name == "__init__"
    )
    assignments = []
    for statement in initializer.body:
        targets = statement.targets if isinstance(statement, ast.Assign) else []
        if any(
            isinstance(target, ast.Attribute)
            and target.attr.startswith(("registration_", "reregistration_"))
            for target in targets
        ):
            assignments.append(statement)
    initialize = ast.FunctionDef(
        name="initialize",
        args=ast.arguments(
            posonlyargs=[],
            args=[ast.arg(arg="self")],
            vararg=None,
            kwonlyargs=[],
            kw_defaults=[],
            kwarg=None,
            defaults=[],
        ),
        body=assignments,
        decorator_list=[],
    )
    module = ast.fix_missing_locations(ast.Module(body=[initialize], type_ignores=[]))
    namespace = {"np": np, "rospy": SimpleNamespace(get_param=get_param)}
    exec(compile(module, str(MISSION_PATH), "exec"), namespace)
    target = SimpleNamespace()
    namespace["initialize"](target)
    return target


def tick_mission_at(phase, x, y, z):
    mission_class = load_mission_class()
    mission = mission_class.__new__(mission_class)
    mission.state_lock = threading.RLock()
    mission.home = (2.0, 3.0, 0.0)
    mission.home_yaw = math.pi / 2.0
    mission.registration_dx = 1.6
    mission.registration_dy = -0.4
    mission.registration_altitude = 1.5
    mission.registration_move_timeout = 30.0
    mission.scan_altitude = 4.0
    mission.waypoint_tolerance = 0.25
    mission.started = FakeTime(100.0)
    mission.phase = phase
    mission.state = SimpleNamespace(connected=True)
    mission.odom = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z))
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0))
        ),
    )
    mission.front_samples = []
    mission.phase_pub = SimpleNamespace(publish=lambda _phase: None)
    mission.publish_command = lambda *_command: None
    return mission


class RegistrationWaypointTest(unittest.TestCase):
    def test_registration_offset_rotates_with_home_heading(self):
        registration_waypoint = load_pure_function("registration_waypoint")
        self.assertIsNotNone(
            registration_waypoint,
            "uav_sphere_mission is missing registration_waypoint",
        )

        x, y = registration_waypoint(2.0, 3.0, math.pi / 2, 1.6, 0.0)

        self.assertAlmostEqual(x, 2.0, places=6)
        self.assertAlmostEqual(y, 4.6, places=6)

    def test_heading_wrap_produces_the_same_waypoint(self):
        registration_waypoint = load_pure_function("registration_waypoint")

        x, y = registration_waypoint(2.0, 3.0, -3.0 * math.pi / 2.0, 1.6, 0.0)

        self.assertAlmostEqual(x, 2.0, places=6)
        self.assertAlmostEqual(y, 4.6, places=6)

    def test_nonzero_body_lateral_offset_is_rotated(self):
        registration_waypoint = load_pure_function("registration_waypoint")

        x, y = registration_waypoint(2.0, 3.0, math.pi / 2.0, 1.6, -0.4)

        self.assertAlmostEqual(x, 2.4, places=6)
        self.assertAlmostEqual(y, 4.6, places=6)

    def test_registration_command_uses_body_relative_xy(self):
        mission_class = load_mission_class()
        mission = mission_class.__new__(mission_class)
        mission.state_lock = threading.RLock()
        mission.home = (2.0, 3.0, 0.0)
        mission.home_yaw = math.pi / 2.0
        mission.registration_dx = 1.6
        mission.registration_dy = -0.4
        mission.registration_offset = 1.6
        mission.registration_altitude = 1.5

        command = mission.registration_command()

        self.assertEqual(len(command), 4)
        self.assertAlmostEqual(command[0], 2.4, places=6)
        self.assertAlmostEqual(command[1], 4.6, places=6)
        self.assertAlmostEqual(command[2], 1.5, places=6)
        self.assertAlmostEqual(command[3], math.pi / 2.0, places=6)

    def test_mission_loads_independent_body_registration_offsets(self):
        requested = []

        def get_param(name, default):
            requested.append((name, default))
            return {"~registration_dx": 1.25, "~registration_dy": -0.75}.get(name, default)

        mission = execute_registration_parameter_assignments(get_param)

        self.assertEqual(getattr(mission, "registration_dx", None), 1.25)
        self.assertEqual(getattr(mission, "registration_dy", None), -0.75)
        self.assertIn(("~registration_dx", 0.60), requested)
        self.assertIn(("~registration_dy", 0.0), requested)

    def test_reregistration_offsets_default_to_registration_offsets(self):
        def get_param(name, default):
            return {"~registration_dx": 1.25, "~registration_dy": -0.75}.get(name, default)

        mission = execute_registration_parameter_assignments(get_param)

        self.assertEqual(getattr(mission, "reregistration_dx", None), 1.25)
        self.assertEqual(getattr(mission, "reregistration_dy", None), -0.75)

    def test_reregistration_offsets_can_be_overridden_independently(self):
        def get_param(name, default):
            return {
                "~registration_dx": 1.25,
                "~registration_dy": -0.75,
                "~reregistration_dx": -0.05,
                "~reregistration_dy": 0.25,
            }.get(name, default)

        mission = execute_registration_parameter_assignments(get_param)

        self.assertEqual(getattr(mission, "registration_dx", None), 1.25)
        self.assertEqual(getattr(mission, "registration_dy", None), -0.75)
        self.assertEqual(getattr(mission, "reregistration_dx", None), -0.05)
        self.assertEqual(getattr(mission, "reregistration_dy", None), 0.25)

    def test_reregistration_command_uses_dedicated_offsets(self):
        mission_class = load_mission_class()
        mission = mission_class.__new__(mission_class)
        mission.state_lock = threading.RLock()
        mission.registration_waypoint = registration_waypoint = load_pure_function(
            "registration_waypoint"
        )
        mission.ugv_odom = SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=5.0, y=6.0, z=1.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )
        mission.transform_point = lambda _target, _source, point: point
        mission.origin_frame = "origin"
        mission.ugv_odom_frame = "ugv_odom"
        mission.uav_odom_frame = "uav_odom"
        # Deliberately different from the re-registration offsets so the test
        # fails if reregistration_command falls back to the initial ones.
        mission.registration_dx = 2.0
        mission.registration_dy = 0.5
        mission.reregistration_dx = -0.05
        mission.reregistration_dy = 0.25
        mission.registration_altitude = 1.8
        mission.home_yaw = 0.7

        command = mission.reregistration_command()

        expected_x, expected_y = registration_waypoint(5.0, 6.0, 0.0, -0.05, 0.25)
        self.assertAlmostEqual(command[0], expected_x, places=6)
        self.assertAlmostEqual(command[1], expected_y, places=6)
        self.assertAlmostEqual(command[2], 1.8, places=6)
        self.assertAlmostEqual(command[3], 0.7, places=6)

    def test_all_registration_phases_use_the_same_body_relative_xy(self):
        mission_class = load_mission_class()
        mission = mission_class.__new__(mission_class)
        mission.state_lock = threading.RLock()
        mission.home = (2.0, 3.0, 0.0)
        mission.home_yaw = math.pi / 2.0
        mission.registration_dx = 1.6
        mission.registration_dy = -0.4
        mission.registration_offset = 1.6
        mission.registration_altitude = 1.5
        mission.scan_altitude = 4.0
        mission.registration_move_timeout = 30.0
        mission.waypoint_tolerance = 0.25
        mission.frozen = False
        mission.scan_index = 0
        mission.scan_step = math.pi / 6.0
        mission.scan_yaw = mission.home_yaw
        mission.scan_step_started = FakeTime(100.0)
        mission.scan_settle_time = 0.8
        mission.scan_dwell = 3.0
        mission.scan_steps = 12
        mission.front_samples = []
        mission.candidate_samples = 3
        mission.candidate_spread = 0.8
        mission.confirmation_samples = 15
        mission.confirmation_spread = 0.35
        mission.confirmation_timeout = 8.0
        mission.started = FakeTime(100.0)
        mission.state = SimpleNamespace(connected=True)
        mission.odom = SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(x=100.0, y=100.0, z=0.0))
            ),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.0, y=0.0, z=0.0)
                )
            ),
        )
        commands = []
        mission.publish_command = lambda *command: commands.append(command)

        phases = (
            "MOVE_TO_REGISTRATION",
            "WAIT_REGISTRATION",
            "CLIMB_FOR_SCAN",
            "FRONT_SCAN",
            "FRONT_CONFIRM",
        )
        for phase in phases:
            mission.phase = phase
            mission.tick(None)

        self.assertEqual(len(commands), len(phases))
        for phase, command in zip(phases, commands):
            with self.subTest(phase=phase):
                self.assertAlmostEqual(command[0], 2.4, places=6)
                self.assertAlmostEqual(command[1], 4.6, places=6)

    def test_move_to_registration_advances_at_rotated_waypoint(self):
        mission = tick_mission_at("MOVE_TO_REGISTRATION", 2.4, 4.6, 1.5)

        mission.tick(None)

        self.assertEqual(mission.phase, "WAIT_REGISTRATION")

    def test_move_to_registration_does_not_advance_at_legacy_world_x_point(self):
        mission = tick_mission_at("MOVE_TO_REGISTRATION", 3.6, 3.0, 1.5)

        mission.tick(None)

        self.assertEqual(mission.phase, "MOVE_TO_REGISTRATION")

    def test_climb_for_scan_advances_at_rotated_waypoint(self):
        mission = tick_mission_at("CLIMB_FOR_SCAN", 2.4, 4.6, 4.0)

        mission.tick(None)

        self.assertEqual(mission.phase, "FRONT_SCAN")

    def test_climb_for_scan_does_not_advance_at_legacy_world_x_point(self):
        mission = tick_mission_at("CLIMB_FOR_SCAN", 3.6, 3.0, 4.0)

        mission.tick(None)

        self.assertEqual(mission.phase, "CLIMB_FOR_SCAN")


if __name__ == "__main__":
    unittest.main()
