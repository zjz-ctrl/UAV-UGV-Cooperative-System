#!/usr/bin/env python3

import ast
from collections import deque
import importlib.util
import math
from pathlib import Path
import statistics
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np


MISSION_PATH = Path(__file__).parents[1] / "scripts" / "uav_sphere_mission.py"
PACKAGE_SRC = Path(__file__).parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from air_ground_bringup.target_handoff import (  # noqa: E402
    DIRECT,
    HOLD,
    REOBSERVE,
    REREGISTER,
    UncertaintyBudget,
    sample_target_covariance,
)


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

    def __eq__(self, other):
        return isinstance(other, FakeTime) and self.seconds == other.seconds


class FakeRospy:
    now_seconds = 100.0

    class Time:
        @staticmethod
        def now():
            return FakeTime(FakeRospy.now_seconds)

    @staticmethod
    def loginfo(*_args):
        pass

    @staticmethod
    def logwarn(*_args):
        pass

    @staticmethod
    def logerr(*_args):
        pass


class FakePoseWithCovarianceStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="", seq=0)
        self.pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            ),
            covariance=[0.0] * 36,
        )


def load_mission_class():
    tree = ast.parse(MISSION_PATH.read_text(), filename=str(MISSION_PATH))
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in ("normalize_angle", "registration_waypoint")
    ]
    mission = next(
        node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Mission"
    )
    namespace = {
        "DIRECT": DIRECT,
        "HOLD": HOLD,
        "REOBSERVE": REOBSERVE,
        "REREGISTER": REREGISTER,
        "UncertaintyBudget": UncertaintyBudget,
        "PoseWithCovarianceStamped": FakePoseWithCovarianceStamped,
        "math": math,
        "np": np,
        "rospy": FakeRospy,
        "sample_target_covariance": sample_target_covariance,
        "statistics": statistics,
        "threading": threading,
    }
    exec(
        compile(ast.Module(body=functions + [mission], type_ignores=[]), str(MISSION_PATH), "exec"),
        namespace,
    )
    return namespace["Mission"]


def odometry(x=0.0, y=0.0, z=0.0, yaw=0.0, speed=0.0, covariance=None):
    pose_covariance = [0.0] * 36 if covariance is None else list(covariance)
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(0.5 * yaw),
                    w=math.cos(0.5 * yaw),
                ),
            ),
            covariance=pose_covariance,
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=speed, y=0.0, z=0.0),
                angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )
        ),
    )


def ros_xyyaw_covariance(values):
    covariance = [0.0] * 36
    for row, source_row in enumerate((0, 1, 5)):
        for column, source_column in enumerate((0, 1, 5)):
            covariance[6 * source_row + source_column] = values[row][column]
    return covariance


class PublisherRecorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class TrackingRLock:
    def __init__(self):
        self.entries = 0
        self.depth = 0

    def __enter__(self):
        self.entries += 1
        self.depth += 1
        return self

    def __exit__(self, _type, _value, _traceback):
        self.depth -= 1


def full_module_ros_stubs(param_overrides=None):
    param_overrides = dict(param_overrides or {})
    publishers = []
    subscribers = []
    timers = []
    requested_params = []

    class Message:
        pass

    class State(Message):
        def __init__(self):
            self.connected = False
            self.mode = ""
            self.armed = False

    class PositionCommand(Message):
        TRAJECTORY_STATUS_READY = 1

    class PoseWithCovarianceStamped(Message):
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="", seq=0)
            self.pose = SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
                ),
                covariance=[0.0] * 36,
            )

    class RegistrationUpdate(Message):
        def __init__(self):
            self.header = SimpleNamespace(seq=0)
            self.revision = 0
            self.pose = SimpleNamespace(covariance=[0.0] * 36)

    class Publisher:
        def __init__(self, topic, message_type, **options):
            self.topic = topic
            self.message_type = message_type
            self.options = options
            self.messages = []
            publishers.append(self)

        def publish(self, message):
            self.messages.append(message)

    class Subscriber:
        def __init__(self, topic, message_type, callback, **options):
            self.topic = topic
            self.message_type = message_type
            self.callback = callback
            self.options = options
            subscribers.append(self)

    class Duration:
        def __init__(self, seconds):
            self.seconds = seconds

    class Time:
        def __init__(self, seconds=0.0):
            self.seconds = seconds

        @staticmethod
        def now():
            return Time(100.0)

    class Timer:
        def __init__(self, duration, callback):
            self.duration = duration
            self.callback = callback
            timers.append(self)

    rospy = SimpleNamespace(
        Duration=Duration,
        Publisher=Publisher,
        ServiceProxy=lambda *_args, **_kwargs: SimpleNamespace(),
        Subscriber=Subscriber,
        Time=Time,
        Timer=Timer,
        get_param=lambda name, default: (
            requested_params.append((name, default)) or param_overrides.get(name, default)
        ),
    )
    tf2_ros = SimpleNamespace(
        Buffer=lambda: SimpleNamespace(),
        TransformListener=lambda buffer: SimpleNamespace(buffer=buffer),
    )

    def module(name, **members):
        result = type(sys)(name)
        result.__dict__.update(members)
        return result

    geometry = module(
        "geometry_msgs.msg",
        PointStamped=type("PointStamped", (Message,), {}),
        PoseStamped=type("PoseStamped", (Message,), {}),
        PoseWithCovarianceStamped=PoseWithCovarianceStamped,
    )
    mavros_msg = module("mavros_msgs.msg", State=State)
    mavros_srv = module(
        "mavros_msgs.srv",
        CommandBool=type("CommandBool", (), {}),
        SetMode=type("SetMode", (), {}),
    )
    nav = module("nav_msgs.msg", Odometry=type("Odometry", (Message,), {}))
    quadrotor = module("quadrotor_msgs.msg", PositionCommand=PositionCommand)
    std = module(
        "std_msgs.msg",
        Bool=type("Bool", (Message,), {}),
        Float64=type("Float64", (Message,), {}),
        String=type("String", (Message,), {}),
    )
    transformations = module(
        "tf.transformations",
        concatenate_matrices=lambda *matrices: np.linalg.multi_dot(matrices),
        euler_from_quaternion=lambda _quaternion: (0.0, 0.0, 0.0),
        quaternion_from_euler=lambda *_angles: (0.0, 0.0, 0.0, 1.0),
        quaternion_matrix=lambda _quaternion: np.eye(4),
        translation_matrix=lambda translation: np.array(
            [
                [1.0, 0.0, 0.0, translation[0]],
                [0.0, 1.0, 0.0, translation[1]],
                [0.0, 0.0, 1.0, translation[2]],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    )
    registration_messages = module(
        "air_ground_coordinate_transform.msg", RegistrationUpdate=RegistrationUpdate
    )
    stubs = {
        "rospy": rospy,
        "tf2_ros": tf2_ros,
        "geometry_msgs": module("geometry_msgs", msg=geometry),
        "geometry_msgs.msg": geometry,
        "mavros_msgs": module("mavros_msgs", msg=mavros_msg, srv=mavros_srv),
        "mavros_msgs.msg": mavros_msg,
        "mavros_msgs.srv": mavros_srv,
        "nav_msgs": module("nav_msgs", msg=nav),
        "nav_msgs.msg": nav,
        "quadrotor_msgs": module("quadrotor_msgs", msg=quadrotor),
        "quadrotor_msgs.msg": quadrotor,
        "std_msgs": module("std_msgs", msg=std),
        "std_msgs.msg": std,
        "tf": module("tf", transformations=transformations),
        "tf.transformations": transformations,
        "air_ground_coordinate_transform": module(
            "air_ground_coordinate_transform", msg=registration_messages
        ),
        "air_ground_coordinate_transform.msg": registration_messages,
    }
    records = SimpleNamespace(
        PoseWithCovarianceStamped=PoseWithCovarianceStamped,
        RegistrationUpdate=RegistrationUpdate,
        publishers=publishers,
        requested_params=requested_params,
        subscribers=subscribers,
        timers=timers,
    )
    return stubs, records


def mission_fixture(phase="FINAL_ESTIMATE"):
    mission_class = load_mission_class()
    mission = mission_class.__new__(mission_class)
    mission.state_lock = threading.RLock()
    mission.phase = phase
    mission.started = FakeTime(100.0)
    mission.state = SimpleNamespace(connected=True)
    mission.home = (0.0, 0.0, 0.0)
    mission.home_yaw = 0.0
    mission.odom = odometry()
    mission.ugv_odom = odometry(10.0, 20.0, 0.0, math.pi / 2.0)
    mission.registration_dx = 2.0
    mission.registration_dy = 1.0
    # Legacy-equivalent: the state machine tests exercise the shared-offset
    # behaviour, so the dedicated re-registration offsets mirror them.
    mission.reregistration_dx = 2.0
    mission.reregistration_dy = 1.0
    mission.registration_altitude = 1.5
    mission.registration_move_timeout = 30.0
    mission.reregistration_timeout = 60.0
    mission.waypoint_tolerance = 0.25
    mission.registration_revision = 3
    mission.baseline_revision = None
    mission.registration_covariance = np.diag([0.04, 0.04, 0.0001])
    mission.uncertainty_aware_handoff = True
    mission.inspection_radius = 0.35
    mission.inspection_yaw = 0.03490658503988659
    mission.target_sigma_floor = 0.02
    mission.final_samples = 2
    mission.final_spread = 0.06
    mission.final_timeout = 12.0
    mission.maximum_camera_disagreement = 0.75
    mission.maximum_sample_speed = 0.15
    mission.maximum_scan_angular_speed = 0.20
    mission.center_altitude = 2.3
    mission.overwatch_altitude = 4.0
    mission.center_target_odom = (3.0, 4.0, 0.25)
    mission.handoff_target_odom = (3.1, 4.1, 0.25)
    mission.approach_yaw = 0.4
    mission.nadir_samples = deque(
        [
            (3.0, 4.0, 0.25, FakeTime(90.0), np.zeros((2, 2))),
            (3.0, 4.0, 0.25, FakeTime(91.0), np.zeros((2, 2))),
        ],
        maxlen=60,
    )
    mission.preserved_target_odom = None
    mission.preserved_target_covariance = None
    mission.preserved_target_stamp = None
    mission.preserved_handoff_target_odom = None
    mission.pending_handoff_action = None
    mission.awaiting_handoff_action = False
    mission.handoff_request_generation = 0
    mission.last_policy_target_stamp = None
    mission.last_policy_target_identity = None
    mission.final_target_origin = None
    mission.final_target_ugv = None
    mission.uav_odom_frame = "iris_0/odom"
    mission.phase_pub = PublisherRecorder()
    mission.anomaly_pub = PublisherRecorder()
    mission.commands = []
    mission.publish_command = lambda *command: mission.commands.append(command)
    mission.publish_last_command = lambda: None
    mission.goal_calls = 0
    mission.dispatch_goal = lambda: setattr(mission, "goal_calls", mission.goal_calls + 1)
    return mission


def final_estimate(spread=0.01, stamp=None):
    selected_stamp = stamp or FakeTime(91.0)
    selected = (
        (3.0, 4.0, 0.25, FakeTime(90.0), np.zeros((2, 2))),
        (3.0, 4.0, 0.25, selected_stamp, np.zeros((2, 2))),
    )
    return (3.0, 4.0, 0.25), spread, selected_stamp, selected


def preserve_and_process(mission, final):
    try:
        mission.preserve_final_estimate(final)
        mission.process_final_estimate()
    except ValueError as error:
        raise AssertionError(
            "final estimate must retain and consume the exact selected sample set"
        ) from error


class RegistrationInputTest(unittest.TestCase):
    def setUp(self):
        mission_class = load_mission_class()
        self.mission = mission_class.__new__(mission_class)
        self.mission.state_lock = threading.RLock()
        self.mission.registration_revision = 2
        self.mission.registration_covariance = np.zeros((3, 3), dtype=float)

    def test_predicted_estimate_maps_ros_xyyaw_covariance_without_revision_change(self):
        self.assertTrue(
            hasattr(self.mission, "registration_estimate_callback"),
            "mission is missing the continuous registration estimate callback",
        )
        covariance = [0.0] * 36
        expected = (
            (0.11, -0.02, 0.03),
            (-0.02, 0.22, -0.04),
            (0.03, -0.04, 0.005),
        )
        for row, source_row in enumerate((0, 1, 5)):
            for column, source_column in enumerate((0, 1, 5)):
                covariance[6 * source_row + source_column] = expected[row][column]
        message = SimpleNamespace(pose=SimpleNamespace(covariance=covariance))

        self.mission.registration_estimate_callback(message)

        np.testing.assert_allclose(self.mission.registration_covariance, expected)
        self.assertEqual(self.mission.registration_revision, 2)

    def test_atomic_update_uses_explicit_revision_and_not_pose_or_header_sequence(self):
        self.assertTrue(
            hasattr(self.mission, "accepted_registration_callback"),
            "mission is missing the atomic accepted-registration callback",
        )
        original_covariance = self.mission.registration_covariance.copy()
        message = SimpleNamespace(
            header=SimpleNamespace(seq=999),
            revision=7,
            pose=SimpleNamespace(covariance=[123.0] * 36),
        )

        self.mission.accepted_registration_callback(message)

        self.assertEqual(self.mission.registration_revision, 7)
        np.testing.assert_array_equal(self.mission.registration_covariance, original_covariance)

        message.header.seq = 1000
        message.revision = 6
        self.mission.accepted_registration_callback(message)
        self.assertEqual(self.mission.registration_revision, 7)


class TargetSampleCovarianceTest(unittest.TestCase):
    def test_projects_full_xyyaw_covariance_at_target_lever_arm(self):
        mission = mission_fixture()
        covariance = ros_xyyaw_covariance([
            [0.4, 0.05, 0.02],
            [0.05, 0.3, -0.01],
            [0.02, -0.01, 0.04],
        ])
        source = odometry(x=10.0, y=20.0, covariance=covariance)
        self.assertTrue(
            hasattr(mission, "target_sample"),
            "camera observations need a validated covariance-bearing sample builder",
        )

        sample = mission.target_sample((12.0, 23.0, 0.25), FakeTime(99.9), source)

        self.assertEqual(sample[:4], (12.0, 23.0, 0.25, FakeTime(99.9)))
        np.testing.assert_allclose(
            sample[4],
            [[0.64, -0.12], [-0.12, 0.42]],
            rtol=0.0,
            atol=1e-12,
        )

    def test_rejects_nonfinite_samples_and_invalid_uav_covariance(self):
        mission = mission_fixture()
        valid = np.diag([0.2, 0.3, 0.04])
        invalid_cases = (
            ((np.nan, 2.0, 0.25), valid),
            ((1.0, np.inf, 0.25), valid),
            ((1.0, 2.0, np.nan), valid),
            ((1.0, 2.0, 0.25), [[np.nan, 0.0, 0.0], [0.0, 0.3, 0.0], [0.0, 0.0, 0.04]]),
            ((1.0, 2.0, 0.25), [[0.2, 0.1, 0.0], [0.0, 0.3, 0.0], [0.0, 0.0, 0.04]]),
            ((1.0, 2.0, 0.25), [[0.2, 0.0, 0.0], [0.0, -0.3, 0.0], [0.0, 0.0, 0.04]]),
        )

        for target, xyyaw_covariance in invalid_cases:
            with self.subTest(target=target, covariance=xyyaw_covariance):
                source = odometry(covariance=ros_xyyaw_covariance(xyyaw_covariance))
                self.assertIsNone(mission.target_sample(target, FakeTime(99.9), source))

    def test_front_and_nadir_callbacks_store_projected_pose_covariance(self):
        mission = mission_fixture("FRONT_APPROACH")
        source = odometry(
            covariance=ros_xyyaw_covariance([
                [0.4, 0.05, 0.02],
                [0.05, 0.3, -0.01],
                [0.02, -0.01, 0.04],
            ])
        )
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=FakeTime(99.9)),
            point=SimpleNamespace(x=2.0, y=3.0, z=0.25),
        )
        mission.frozen = True
        mission.front_samples = deque(maxlen=60)
        mission.nearest_odom = lambda _stamp: source
        mission.pose_matrix = lambda _pose: np.eye(4)
        mission.body_from_front = np.eye(4)
        mission.body_from_nadir = np.eye(4)
        mission.front_minimum_range = 0.0
        mission.front_maximum_range = 10.0
        mission.front_height_tolerance = 0.1
        mission.nadir_maximum_range = 5.0
        mission.publish_diagnostic_point = lambda *_args: None
        mission.front_odom_pub = PublisherRecorder()
        mission.nadir_odom_pub = PublisherRecorder()
        mission.ball_plane_height = lambda: 0.25

        mission.front_point_callback(message)
        mission.ball_plane_height = lambda: -1.0
        message.point.z = -1.0
        mission.nadir_ray_callback(message)

        self.assertEqual(len(mission.front_samples), 1)
        self.assertEqual(len(mission.nadir_samples), 3)
        np.testing.assert_allclose(
            mission.front_samples[-1][4],
            [[0.64, -0.12], [-0.12, 0.42]],
            rtol=0.0,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            mission.nadir_samples[-1][4],
            [[0.64, -0.12], [-0.12, 0.42]],
            rtol=0.0,
            atol=1e-12,
        )

    def test_selected_window_drives_unbiased_covariance_floor_pose_and_stamp_once(self):
        pose_covariances = (
            np.array([[0.1, 0.02], [0.02, 0.2]]),
            np.array([[0.3, 0.04], [0.04, 0.4]]),
            np.array([[0.5, 0.06], [0.06, 0.6]]),
        )
        samples = deque([
            (100.0, 100.0, 0.25, FakeTime(99.6), np.eye(2)),
            (0.0, 0.0, 0.25, FakeTime(99.7), pose_covariances[0]),
            (2.0, 0.0, 0.25, FakeTime(99.8), pose_covariances[1]),
            (0.0, 2.0, 0.25, FakeTime(99.9), pose_covariances[2]),
        ])
        mission = mission_fixture()
        mission.target_sigma_floor = 0.5

        final = mission.stable_target(samples, 3, 2.1)

        self.assertEqual(final[:3], ((0.0, 0.0, 0.25), 2.0, FakeTime(99.9)))
        self.assertEqual(len(final[3]), 3)
        for actual, expected in zip(final[3], tuple(samples)[-3:]):
            self.assertIs(actual, expected)
        self.assertTrue(mission.preserve_final_estimate(final))
        np.testing.assert_allclose(
            mission.preserved_target_covariance,
            [[1.8833333333333333, -0.6266666666666666],
             [-0.6266666666666666, 1.9833333333333334]],
            rtol=0.0,
            atol=1e-12,
        )
        self.assertEqual(mission.preserved_target_stamp, FakeTime(99.9))

    def test_invalid_selected_covariance_is_not_preserved(self):
        mission = mission_fixture()
        selected = (
            (3.0, 4.0, 0.25, FakeTime(99.8), np.eye(2)),
            (3.1, 4.1, 0.25, FakeTime(99.9), np.array([[1.0, 0.2], [0.0, 1.0]])),
        )

        preserved = mission.preserve_final_estimate(
            ((3.05, 4.05, 0.25), 0.1, FakeTime(99.9), selected)
        )

        self.assertFalse(preserved)
        self.assertIsNone(mission.preserved_target_odom)
        self.assertIsNone(mission.preserved_target_covariance)


class AnomalyPublicationTest(unittest.TestCase):
    def test_message_uses_selected_stamp_uav_frame_identity_and_only_xy_covariance(self):
        mission = mission_fixture()
        mission.preserved_target_odom = (3.0, 4.0, 0.25)
        mission.preserved_target_stamp = FakeTime(99.9)
        mission.preserved_target_covariance = np.array([[0.11, -0.03], [-0.03, 0.22]])
        mission.pending_handoff_action = DIRECT
        mission.handoff_request_generation = 4
        mission.registration_covariance = np.diag([100.0, 200.0, 300.0])
        self.assertTrue(
            hasattr(mission, "publish_anomaly_estimate"),
            "mission needs a covariance-bearing anomaly publisher",
        )

        mission.publish_anomaly_estimate()

        self.assertEqual(len(mission.anomaly_pub.messages), 1)
        message = mission.anomaly_pub.messages[0]
        self.assertEqual(message.header.frame_id, "iris_0/odom")
        self.assertEqual(message.header.stamp, FakeTime(99.9))
        self.assertEqual(message.header.seq, 0)
        self.assertEqual(
            (message.pose.pose.position.x, message.pose.pose.position.y,
             message.pose.pose.position.z),
            (3.0, 4.0, 0.25),
        )
        self.assertEqual(
            (message.pose.pose.orientation.x, message.pose.pose.orientation.y,
             message.pose.pose.orientation.z, message.pose.pose.orientation.w),
            (0.0, 0.0, 0.0, 1.0),
        )
        expected_covariance = [0.0] * 36
        expected_covariance[0] = 0.11
        expected_covariance[1] = -0.03
        expected_covariance[6] = -0.03
        expected_covariance[7] = 0.22
        self.assertEqual(message.pose.covariance, expected_covariance)
        self.assertEqual(mission.handoff_request_generation, 5)
        self.assertTrue(mission.awaiting_handoff_action)
        self.assertIsNone(mission.pending_handoff_action)

        mission.registration_covariance[:] = np.nan
        mission.publish_anomaly_estimate()
        self.assertEqual(
            mission.anomaly_pub.messages[1].pose.covariance,
            expected_covariance,
            "registration covariance must not enter the UAV-frame anomaly",
        )


class FullMissionInitializationTest(unittest.TestCase):
    def test_enabled_constructor_wires_anomaly_producer_and_action_consumer_ownership(self):
        stubs, records = full_module_ros_stubs({"~uncertainty_aware_handoff": True})
        module_name = "task9_full_uav_sphere_mission"
        specification = importlib.util.spec_from_file_location(module_name, MISSION_PATH)
        mission_module = importlib.util.module_from_spec(specification)

        with patch.dict(sys.modules, stubs):
            specification.loader.exec_module(mission_module)
            mission = mission_module.Mission()

        defaults = dict(records.requested_params)
        self.assertIs(defaults["~uncertainty_aware_handoff"], False)
        self.assertEqual(defaults["~inspection_radius"], 0.35)
        self.assertEqual(defaults["~inspection_yaw"], 0.03490658503988659)
        self.assertEqual(defaults["~target_sigma_floor"], 0.02)
        self.assertEqual(defaults["~reregistration_timeout"], 60.0)
        self.assertIsInstance(mission.state_lock, type(threading.RLock()))

        publishers = {publisher.topic: publisher for publisher in records.publishers}
        anomaly = publishers["/air_ground/anomaly/uav_estimate"]
        self.assertEqual(
            anomaly.message_type.__name__, "PoseWithCovarianceStamped"
        )
        self.assertEqual(anomaly.options, {"queue_size": 1, "latch": True})
        self.assertNotIn("/air_ground/handoff/action", publishers)
        self.assertNotIn("/air_ground/handoff/confidence_radius", publishers)

        subscribers = {subscriber.topic: subscriber for subscriber in records.subscribers}
        accepted = subscribers["/air_ground/registration/accepted_update"]
        estimate = subscribers["/air_ground/registration/estimate"]
        action = subscribers["/air_ground/handoff/action"]
        self.assertIs(accepted.message_type, records.RegistrationUpdate)
        self.assertEqual(accepted.callback.__name__, "accepted_registration_callback")
        self.assertIs(accepted.callback.__self__, mission)
        self.assertIs(estimate.message_type, records.PoseWithCovarianceStamped)
        self.assertEqual(estimate.callback.__name__, "registration_estimate_callback")
        self.assertIs(estimate.callback.__self__, mission)
        self.assertEqual(action.message_type.__name__, "String")
        self.assertEqual(action.callback.__name__, "handoff_action_callback")
        self.assertIs(action.callback.__self__, mission)
        self.assertEqual(action.options, {"queue_size": 1})
        self.assertFalse(action.options.get("latch", False))
        self.assertEqual(mission.handoff_request_generation, 0)
        self.assertFalse(mission.awaiting_handoff_action)

        self.assertEqual(len(records.timers), 1)
        self.assertAlmostEqual(records.timers[0].duration.seconds, 1.0 / 30.0)
        self.assertEqual(records.timers[0].callback.__name__, "tick")
        self.assertIs(records.timers[0].callback.__self__, mission)

        covariance_message = records.PoseWithCovarianceStamped()
        covariance_message.pose.covariance[0] = 0.11
        covariance_message.pose.covariance[7] = 0.22
        covariance_message.pose.covariance[35] = 0.005
        estimate.callback(covariance_message)
        revision = records.RegistrationUpdate()
        revision.header.seq = 900
        revision.revision = 7
        accepted.callback(revision)
        revision.header.seq = 901
        revision.revision = 6
        accepted.callback(revision)

        np.testing.assert_array_equal(
            mission.registration_covariance,
            [[0.11, 0.0, 0.0], [0.0, 0.22, 0.0], [0.0, 0.0, 0.005]],
        )
        self.assertEqual(mission.registration_revision, 7)


class ReregistrationStateMachineTest(unittest.TestCase):
    def setUp(self):
        FakeRospy.now_seconds = 100.0

    @staticmethod
    def install_transform_chain(mission):
        calls = []

        def transform(target_frame, source_frame, point):
            calls.append((target_frame, source_frame, tuple(point)))
            if (target_frame, source_frame) == ("air_ground_origin", "ugv_0/odom"):
                return point[0] + 100.0, point[1] + 200.0, point[2]
            if (target_frame, source_frame) == ("iris_0/odom", "air_ground_origin"):
                return point[0] - 10.0, point[1] - 20.0, point[2]
            return None

        mission.origin_frame = "air_ground_origin"
        mission.uav_odom_frame = "iris_0/odom"
        mission.ugv_odom_frame = "ugv_0/odom"
        mission.transform_point = transform
        return calls

    def test_rendezvous_uses_latest_ugv_body_waypoint_through_both_transforms(self):
        mission = mission_fixture("RETURN_TO_UGV")
        self.assertTrue(
            hasattr(mission, "reregistration_command"),
            "mission is missing the dynamic re-registration command",
        )
        calls = self.install_transform_chain(mission)

        first = mission.reregistration_command()
        mission.ugv_odom = odometry(20.0, 30.0, 0.0, 0.0)
        second = mission.reregistration_command()

        self.assertEqual(first, (99.0, 202.0, 1.5, 0.0))
        self.assertEqual(second, (112.0, 211.0, 1.5, 0.0))
        self.assertEqual(
            calls,
            [
                ("air_ground_origin", "ugv_0/odom", (9.0, 22.0, 0.0)),
                ("iris_0/odom", "air_ground_origin", (109.0, 222.0, 0.0)),
                ("air_ground_origin", "ugv_0/odom", (22.0, 31.0, 0.0)),
                ("iris_0/odom", "air_ground_origin", (122.0, 231.0, 0.0)),
            ],
        )

    def test_return_wait_resume_sequence_captures_wait_entry_revision(self):
        mission = mission_fixture()
        self.install_transform_chain(mission)
        mission.publish_final_target = lambda _target: self.fail(
            "enabled handoff must never resolve the legacy target"
        )
        preserve_and_process(mission, final_estimate())
        self.assertEqual(mission.phase, "FINAL_ESTIMATE")
        self.assertEqual(len(mission.anomaly_pub.messages), 1)

        mission.handoff_action_callback(SimpleNamespace(data=REREGISTER))
        self.assertEqual(mission.phase, "RETURN_TO_UGV")

        mission.accepted_registration_callback(SimpleNamespace(revision=4))
        mission.odom = odometry(99.0, 202.0, 1.5, speed=0.15)
        mission.tick(None)

        self.assertEqual(mission.phase, "WAIT_REREGISTRATION")
        self.assertEqual(mission.baseline_revision, 4)
        self.assertEqual(mission.goal_calls, 0)

        for revision in (4, 3):
            mission.registration_revision = revision
            mission.tick(None)
            self.assertEqual(mission.phase, "WAIT_REREGISTRATION")
            self.assertEqual(mission.goal_calls, 0)

        mission.registration_revision = 5
        mission.tick(None)
        self.assertEqual(mission.phase, "RESUME_HANDOFF")
        self.assertEqual(mission.goal_calls, 0)

        mission.stable_target = lambda *_args: self.fail("resume must not rerun target detection")
        mission.tick(None)

        self.assertEqual(mission.phase, "RESUME_HANDOFF")
        self.assertEqual(len(mission.anomaly_pub.messages), 2)
        self.assertTrue(mission.awaiting_handoff_action)
        mission.tick(None)
        self.assertEqual(len(mission.anomaly_pub.messages), 2)
        mission.handoff_action_callback(SimpleNamespace(data=DIRECT))
        self.assertEqual(mission.phase, "DISPATCH")
        self.assertEqual(mission.goal_calls, 0)

    def test_arrival_requires_xy_altitude_and_safe_speed(self):
        mission = mission_fixture("RETURN_TO_UGV")
        self.install_transform_chain(mission)
        unsafe_cases = (
            odometry(99.3, 202.0, 1.5, speed=0.0),
            odometry(99.0, 202.0, 1.8, speed=0.0),
            odometry(99.0, 202.0, 1.5, speed=0.151),
        )

        for current_odom in unsafe_cases:
            with self.subTest(odom=current_odom):
                mission.phase = "RETURN_TO_UGV"
                mission.started = FakeTime(100.0)
                mission.odom = current_odom
                mission.tick(None)
                self.assertEqual(mission.phase, "RETURN_TO_UGV")

    def test_return_and_wait_have_distinct_timeouts(self):
        mission = mission_fixture("RETURN_TO_UGV")
        self.install_transform_chain(mission)
        FakeRospy.now_seconds = 131.0
        mission.tick(None)
        self.assertEqual(mission.phase, "ERROR_REGISTRATION")

    def test_wait_requires_current_rendezvous_before_accepting_new_revision(self):
        mission = mission_fixture("WAIT_REREGISTRATION")
        mission.baseline_revision = 3
        mission.registration_revision = 4
        mission.origin_frame = "air_ground_origin"
        mission.uav_odom_frame = "iris_0/odom"
        mission.ugv_odom_frame = "ugv_0/odom"
        mission.transform_point = lambda *_args: None

        mission.tick(None)

        self.assertEqual(mission.phase, "WAIT_REREGISTRATION")

        FakeRospy.now_seconds = 161.0
        mission.tick(None)
        self.assertEqual(mission.phase, "ERROR_REGISTRATION")

    def test_deadline_expiry_precedes_late_arrival_or_revision(self):
        returning = mission_fixture("RETURN_TO_UGV")
        self.install_transform_chain(returning)
        returning.odom = odometry(99.0, 202.0, 1.5, speed=0.0)
        FakeRospy.now_seconds = 131.0
        returning.tick(None)
        self.assertEqual(returning.phase, "ERROR_REGISTRATION")

        waiting = mission_fixture("WAIT_REREGISTRATION")
        self.install_transform_chain(waiting)
        waiting.baseline_revision = 3
        waiting.registration_revision = 4
        FakeRospy.now_seconds = 161.0
        waiting.tick(None)
        self.assertEqual(waiting.phase, "ERROR_REGISTRATION")

    def test_exact_deadline_boundary_still_allows_success(self):
        returning = mission_fixture("RETURN_TO_UGV")
        self.install_transform_chain(returning)
        returning.odom = odometry(99.0, 202.0, 1.5, speed=0.0)
        FakeRospy.now_seconds = 130.0
        returning.tick(None)
        self.assertEqual(returning.phase, "WAIT_REREGISTRATION")

        waiting = mission_fixture("WAIT_REREGISTRATION")
        self.install_transform_chain(waiting)
        waiting.baseline_revision = 3
        waiting.registration_revision = 4
        FakeRospy.now_seconds = 160.0
        waiting.tick(None)
        self.assertEqual(waiting.phase, "RESUME_HANDOFF")

        mission = mission_fixture("WAIT_REREGISTRATION")
        self.install_transform_chain(mission)
        mission.baseline_revision = 3
        FakeRospy.now_seconds = 161.0
        mission.tick(None)
        self.assertEqual(mission.phase, "ERROR_REGISTRATION")

    def test_missing_ugv_or_transform_holds_return_and_resume_waits_for_relay(self):
        for missing in ("odom", "transform"):
            with self.subTest(missing=missing):
                mission = mission_fixture("RETURN_TO_UGV")
                if missing == "odom":
                    mission.ugv_odom = None
                else:
                    mission.transform_point = lambda *_args: None
                    mission.origin_frame = "air_ground_origin"
                    mission.uav_odom_frame = "iris_0/odom"
                    mission.ugv_odom_frame = "ugv_0/odom"
                mission.tick(None)
                self.assertEqual(mission.phase, "RETURN_TO_UGV")
                self.assertEqual(mission.commands, [])
                self.assertEqual(mission.goal_calls, 0)

        mission = mission_fixture("RESUME_HANDOFF")
        preserved = (3.0, 4.0, 0.25)
        mission.preserved_target_odom = preserved
        mission.preserved_target_covariance = np.diag([0.1, 0.2])
        mission.preserved_target_stamp = FakeTime(99.9)
        mission.publish_final_target = lambda _target: self.fail(
            "resume must not use the legacy transform path"
        )
        mission.tick(None)
        self.assertEqual(mission.phase, "RESUME_HANDOFF")
        self.assertEqual(mission.preserved_target_odom, preserved)
        self.assertEqual(len(mission.anomaly_pub.messages), 1)
        self.assertTrue(mission.awaiting_handoff_action)
        self.assertEqual(mission.goal_calls, 0)

    def test_reregister_preserves_complete_target_before_return(self):
        mission = mission_fixture()
        mission.publish_final_target = lambda _target: self.fail(
            "REREGISTER must defer target resolution until resume"
        )
        final = final_estimate()
        self.assertTrue(
            hasattr(mission, "process_final_estimate"),
            "mission is missing uncertainty-aware final-estimate handling",
        )

        preserve_and_process(mission, final)
        mission.handoff_action_callback(SimpleNamespace(data=REREGISTER))

        self.assertEqual(mission.phase, "RETURN_TO_UGV")
        self.assertEqual(mission.preserved_target_odom, final[0])
        np.testing.assert_array_equal(
            mission.preserved_target_covariance,
            [[0.0004, 0.0], [0.0, 0.0004]],
        )
        self.assertEqual(mission.preserved_target_stamp, FakeTime(91.0))
        self.assertEqual(mission.preserved_handoff_target_odom, (3.1, 4.1, 0.25))
        self.assertEqual(len(mission.anomaly_pub.messages), 1)
        self.assertEqual(mission.pending_handoff_action, REREGISTER)
        self.assertFalse(mission.awaiting_handoff_action)
        self.assertEqual(mission.goal_calls, 0)

    def test_direct_reobserve_and_hold_have_distinct_safe_effects(self):
        direct = mission_fixture()
        self.assertTrue(
            hasattr(direct, "process_final_estimate"),
            "mission is missing uncertainty-aware final-estimate handling",
        )
        direct.publish_final_target = lambda _target: self.fail(
            "DIRECT from the relay must not invoke legacy target publication"
        )
        preserve_and_process(direct, final_estimate())
        direct.handoff_action_callback(SimpleNamespace(data=DIRECT))
        self.assertEqual(direct.phase, "DISPATCH")
        self.assertEqual(direct.pending_handoff_action, DIRECT)
        self.assertFalse(direct.awaiting_handoff_action)

        reobserve = mission_fixture()
        preserve_and_process(reobserve, final_estimate(spread=0.2))
        reobserve.handoff_action_callback(SimpleNamespace(data=REOBSERVE))
        self.assertEqual(reobserve.phase, "CENTER_OVER_SPHERE")
        self.assertEqual(list(reobserve.nadir_samples), [])
        self.assertEqual(reobserve.pending_handoff_action, REOBSERVE)
        self.assertFalse(reobserve.awaiting_handoff_action)

        hold = mission_fixture()
        hold.publish_final_target = lambda _target: self.fail("HOLD must not resolve a goal")
        final = final_estimate()
        preserve_and_process(hold, final)
        hold.handoff_action_callback(SimpleNamespace(data=HOLD))
        self.assertEqual(hold.phase, "FINAL_ESTIMATE")
        self.assertEqual(hold.preserved_target_odom, final[0])
        self.assertEqual(hold.pending_handoff_action, HOLD)
        self.assertTrue(hold.awaiting_handoff_action)
        self.assertEqual(hold.goal_calls, 0)

        hold.handoff_action_callback(SimpleNamespace(data=DIRECT))
        self.assertEqual(hold.phase, "DISPATCH")
        self.assertFalse(hold.awaiting_handoff_action)

    def test_disabled_uncertainty_policy_retains_legacy_direct_dispatch(self):
        mission = mission_fixture()
        self.assertTrue(
            hasattr(mission, "process_final_estimate"),
            "mission is missing final-estimate handling",
        )
        mission.uncertainty_aware_handoff = False
        mission.registration_covariance[:] = np.nan
        published = []
        mission.publish_final_target = lambda target: published.append(target) or True

        preserve_and_process(mission, final_estimate())

        self.assertEqual(mission.phase, "DISPATCH")
        self.assertEqual(published, [(3.0, 4.0, 0.25)])
        self.assertEqual(mission.anomaly_pub.messages, [])

        mission.handoff_action_callback(SimpleNamespace(data=REOBSERVE))
        self.assertEqual(mission.phase, "DISPATCH")
        mission.tick(None)
        self.assertEqual(mission.goal_calls, 1)

    def test_out_of_phase_or_nonawaited_actions_are_ignored(self):
        mission = mission_fixture()
        final = final_estimate()

        mission.preserve_final_estimate(final)
        mission.handoff_action_callback(SimpleNamespace(data=DIRECT))
        self.assertEqual(mission.phase, "FINAL_ESTIMATE")

        preserve_and_process(mission, final)
        mission.handoff_action_callback(SimpleNamespace(data=REREGISTER))
        self.assertEqual(mission.phase, "RETURN_TO_UGV")
        self.assertFalse(mission.awaiting_handoff_action)

        mission.handoff_action_callback(SimpleNamespace(data=DIRECT))
        self.assertEqual(mission.phase, "RETURN_TO_UGV")
        mission.phase = "CENTER_OVER_SPHERE"
        mission.awaiting_handoff_action = True
        mission.handoff_action_callback(SimpleNamespace(data=DIRECT))
        self.assertEqual(mission.phase, "CENTER_OVER_SPHERE")

    def test_registration_changes_do_not_run_a_mission_side_policy(self):
        mission = mission_fixture()
        final = final_estimate()
        preserve_and_process(mission, final)
        self.assertEqual(mission.phase, "FINAL_ESTIMATE")
        self.assertEqual(len(mission.anomaly_pub.messages), 1)

        mission.registration_covariance[:] = np.nan
        mission.process_final_estimate()

        self.assertEqual(mission.phase, "FINAL_ESTIMATE")
        self.assertTrue(mission.awaiting_handoff_action)
        self.assertEqual(len(mission.anomaly_pub.messages), 1)

    def test_enabled_dispatch_advances_without_legacy_goal_publication(self):
        mission = mission_fixture("DISPATCH")

        mission.tick(None)

        self.assertEqual(mission.goal_calls, 0)
        self.assertEqual(mission.phase, "OVERWATCH")

    def test_stable_window_timestamp_remains_canonical_after_later_append(self):
        samples = deque(
            [
                (2.8, 3.8, 0.25, FakeTime(99.8), np.zeros((2, 2))),
                (3.0, 4.0, 0.25, FakeTime(99.9), np.zeros((2, 2))),
            ],
            maxlen=60,
        )
        mission = mission_fixture()

        final = mission.stable_target(samples, 2, 1.0)

        self.assertEqual(len(final), 4, "stable target must include the exact selected window")
        self.assertEqual(final[2], FakeTime(99.9))
        selected = final[3]
        samples.append((100.0, 100.0, 0.25, FakeTime(100.0), np.eye(2)))
        mission.nadir_samples = samples
        preserve_and_process(mission, final)
        self.assertEqual(mission.preserved_target_stamp, FakeTime(99.9))
        self.assertEqual(final[3], selected)

    def test_final_tick_preserves_selected_result_exactly_once(self):
        mission = mission_fixture()
        final = final_estimate()
        mission.stable_target = lambda *_args: final
        preservation_calls = []

        def preserve(selected):
            preservation_calls.append(selected)
            mission.preserved_target_odom = selected[0]
            mission.preserved_target_covariance = np.diag([0.0004, 0.0004])
            mission.preserved_target_stamp = selected[2]
            mission.preserved_handoff_target_odom = mission.handoff_target_odom
            return True

        mission.preserve_final_estimate = preserve
        mission.publish_final_target = lambda _target: True

        mission.tick(None)
        mission.tick(None)

        self.assertEqual(preservation_calls, [final])
        self.assertEqual(len(mission.anomaly_pub.messages), 1)

    def test_disagreement_error_preserves_immutable_final_before_transition(self):
        mission = mission_fixture()
        final = final_estimate(spread=0.03, stamp=FakeTime(99.9))
        mission.stable_target = lambda *_args: final
        mission.handoff_target_odom = (30.0, 40.0, 0.25)
        events = []
        preserve = mission.preserve_final_estimate
        set_phase = mission.set_phase

        def record_preservation(selected):
            events.append(("preserve", selected))
            preserve(selected)

        def record_phase(phase):
            events.append(("phase", phase))
            set_phase(phase)

        mission.preserve_final_estimate = record_preservation
        mission.set_phase = record_phase

        mission.tick(None)

        self.assertEqual(mission.phase, "ERROR_COORDINATE")
        self.assertEqual(
            events,
            [("preserve", final), ("phase", "ERROR_COORDINATE")],
        )
        self.assertEqual(mission.preserved_target_odom, final[0])
        np.testing.assert_array_equal(
            mission.preserved_target_covariance,
            [[0.0004, 0.0], [0.0, 0.0004]],
        )
        self.assertEqual(mission.preserved_target_stamp, final[2])

    def test_task8_callbacks_phase_and_tick_enter_one_shared_lock(self):
        mission = mission_fixture("WAIT_REREGISTRATION")
        tracker = TrackingRLock()
        mission.state_lock = tracker
        mission.baseline_revision = 3
        self.install_transform_chain(mission)
        covariance = [0.0] * 36
        estimate = SimpleNamespace(pose=SimpleNamespace(covariance=covariance))

        mission.registration_estimate_callback(estimate)
        mission.accepted_registration_callback(SimpleNamespace(revision=4))
        mission.awaiting_handoff_action = True
        mission.handoff_action_callback(SimpleNamespace(data=HOLD))
        mission.set_phase("RESUME_HANDOFF")
        mission.phase = "WAIT_REREGISTRATION"
        mission.tick(None)

        self.assertGreaterEqual(tracker.entries, 5)
        self.assertEqual(tracker.depth, 0)


if __name__ == "__main__":
    unittest.main()
