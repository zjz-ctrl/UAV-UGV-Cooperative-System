#!/usr/bin/env python3

import ast
import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import numpy as np

from air_ground_coordinate_transform.odom_buffer import OdomBuffer
from air_ground_coordinate_transform.registration_coordinator import RegistrationCoordinator
from air_ground_coordinate_transform.registration_estimator import (
    RegistrationFilter,
    RegistrationSample,
    RobustBatchEstimator,
)


class Stamp:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def to_sec(self):
        return self.seconds

    def is_zero(self):
        return self.seconds == 0.0


class Publisher:
    def __init__(self, name, events=None, callback=None):
        self.name = name
        self.messages = []
        self.events = events
        self.callback = callback

    def publish(self, message):
        self.messages.append(message)
        if self.events is not None:
            self.events.append(self.name)
        if self.callback is not None:
            self.callback(message)


class PoseWithCovarianceStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="", seq=0)
        self.pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=[0.0] * 36,
        )


class TransformStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.child_frame_id = ""
        self.transform = SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


class RegistrationUpdate:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="", seq=0)
        self.revision = 0
        self.pose = PoseWithCovarianceStamped().pose


PLANAR_COVARIANCE_INDICES = (0, 1, 5, 6, 7, 11, 30, 31, 35)


def pose_components(pose_with_covariance):
    pose = pose_with_covariance.pose
    return (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


def message_module(name, **messages):
    package = ModuleType(name)
    child = ModuleType(name + ".msg")
    for message_name, message_type in messages.items():
        setattr(child, message_name, message_type)
    package.msg = child
    return package, child


def load_registration_module(now=11.05):
    rospy = ModuleType("rospy")
    rospy.Time = SimpleNamespace(
        now=lambda: Stamp(now), from_sec=lambda seconds: Stamp(seconds)
    )
    rospy.logwarn_throttle = lambda *_args: None
    rospy.loginfo = lambda *_args: None
    geometry, geometry_msg = message_module(
        "geometry_msgs",
        PoseWithCovarianceStamped=PoseWithCovarianceStamped,
        TransformStamped=TransformStamped,
    )
    nav, nav_msg = message_module("nav_msgs", Odometry=object)
    std, std_msg = message_module(
        "std_msgs", Bool=object, Float64=object, String=object, UInt32=object
    )
    registration_messages = ModuleType("air_ground_coordinate_transform.msg")
    registration_messages.RegistrationUpdate = RegistrationUpdate
    transformations = ModuleType("tf.transformations")
    transformations.concatenate_matrices = lambda *matrices: np.linalg.multi_dot(matrices)
    transformations.euler_from_quaternion = lambda _quaternion: (0.0, 0.0, 0.0)
    transformations.quaternion_from_euler = lambda _roll, _pitch, yaw: (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )
    transformations.quaternion_from_matrix = lambda _matrix: (0.0, 0.0, 0.0, 1.0)
    transformations.quaternion_matrix = lambda _quaternion: np.eye(4)
    transformations.translation_matrix = lambda translation: np.array(
        [
            [1.0, 0.0, 0.0, translation[0]],
            [0.0, 1.0, 0.0, translation[1]],
            [0.0, 0.0, 1.0, translation[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    tf = ModuleType("tf")
    tf.transformations = transformations
    tf2_ros = ModuleType("tf2_ros")
    tf2_ros.TransformBroadcaster = object
    replacements = {
        "rospy": rospy,
        "geometry_msgs": geometry,
        "geometry_msgs.msg": geometry_msg,
        "nav_msgs": nav,
        "nav_msgs.msg": nav_msg,
        "std_msgs": std,
        "std_msgs.msg": std_msg,
        "air_ground_coordinate_transform.msg": registration_messages,
        "tf": tf,
        "tf.transformations": transformations,
        "tf2_ros": tf2_ros,
    }
    script = Path(__file__).parents[1] / "scripts" / "takeoff_registration.py"
    specification = importlib.util.spec_from_file_location(
        "takeoff_registration_under_test", script
    )
    module = importlib.util.module_from_spec(specification)
    with mock.patch.dict(sys.modules, replacements):
        specification.loader.exec_module(module)
    return module


def make_coordinator():
    rates = {
        "translation_time_variance_rate": 0.1,
        "translation_uav_distance_variance_rate": 0.2,
        "translation_ugv_distance_variance_rate": 0.3,
        "yaw_time_variance_rate": 0.01,
        "yaw_uav_distance_variance_rate": 0.02,
        "yaw_ugv_distance_variance_rate": 0.03,
    }
    return RegistrationCoordinator(
        mode="opportunistic",
        registration_filter=RegistrationFilter(None, None, rates),
        estimator=RobustBatchEstimator(1, 0.05, 0.03),
        registration_window_seconds=2.0,
        registration_window_max_samples=6,
        sample_period=0.01,
        periodic_update_seconds=10.0,
        degraded_covariance_trace_threshold=100.0,
        innovation_mahalanobis_threshold=11.344866730144373,
        max_batch_coalesce_age=0.1,
    )


def registration_sample(mean, stamp):
    return RegistrationSample(
        mean=np.asarray(mean, dtype=float),
        anchor=np.zeros(2),
        covariance=np.diag([1e-6, 1e-6, 1e-7]),
        stamp=float(stamp),
    )


def odometry_message(
    stamp,
    x,
    frame_id="expected_parent",
    child_frame_id="expected_child",
    z=2.0,
    twist_x=0.0,
):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=Stamp(stamp), frame_id=frame_id),
        child_frame_id=child_frame_id,
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=0.0, z=z),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=twist_x, y=0.0, z=0.0),
                angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )
        ),
    )


class RegistrationNodeAdapterTest(unittest.TestCase):
    def test_constructor_creates_latched_continuous_registration_state_publisher(self):
        module = load_registration_module()
        publishers = []
        required_params = {
            "~uav_base_to_camera_translation": [0.0, 0.0, 0.0],
            "~uav_base_to_camera_rpy": [0.0, 0.0, 0.0],
            "~ugv_base_to_board_translation": [0.0, 0.0, 0.0],
            "~ugv_base_to_board_rpy": [0.0, 0.0, 0.0],
            "~uav_odom_topic": "/uav/odom",
            "~ugv_odom_topic": "/ugv/odom",
            "~observation_topic": "/board/pose",
        }
        missing = object()

        def get_param(name, default=missing):
            if name in required_params:
                return required_params[name]
            param_name = name[1:] if name.startswith("~") else name
            if param_name in RegistrationFilter._PROCESS_NOISE_NAMES:
                return 0.0
            if default is not missing:
                return default
            raise AssertionError("Unexpected required parameter: {}".format(name))

        def publisher(topic, message_type, queue_size, latch=False):
            publishers.append((topic, message_type, queue_size, latch))
            return Publisher(topic)

        module.rospy.get_param = get_param
        module.rospy.Publisher = publisher
        module.rospy.Subscriber = lambda *_args, **_kwargs: None
        module.rospy.Timer = lambda *_args, **_kwargs: None
        module.rospy.Duration = lambda seconds: seconds
        module.TransformBroadcaster = lambda: object()

        module.Registration()

        state_publishers = [
            details
            for details in publishers
            if details[0] == "/air_ground/registration/state"
        ]
        self.assertEqual(
            state_publishers,
            [
                (
                    "/air_ground/registration/state",
                    module.RegistrationUpdate,
                    1,
                    True,
                )
            ],
        )

    def test_registration_update_message_has_explicit_revision_contract(self):
        message_path = Path(__file__).parents[1] / "msg" / "RegistrationUpdate.msg"

        self.assertTrue(message_path.is_file(), "RegistrationUpdate.msg is missing")
        fields = [
            line.strip()
            for line in message_path.read_text().splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]
        self.assertEqual(
            fields,
            [
                "std_msgs/Header header",
                "uint32 revision",
                "geometry_msgs/PoseWithCovariance pose",
            ],
        )

    def test_registration_update_generation_and_consumer_dependencies_are_declared(self):
        coordinate_package = Path(__file__).parents[1]
        workspace = coordinate_package.parent
        bringup_package = workspace / "air_ground_bringup"
        coordinate_cmake = (coordinate_package / "CMakeLists.txt").read_text()
        coordinate_manifest = ET.parse(coordinate_package / "package.xml").getroot()
        bringup_cmake = (bringup_package / "CMakeLists.txt").read_text()
        bringup_manifest = ET.parse(bringup_package / "package.xml").getroot()

        self.assertIn("message_generation", coordinate_cmake)
        self.assertIn("add_message_files", coordinate_cmake)
        self.assertIn("RegistrationUpdate.msg", coordinate_cmake)
        self.assertIn("generate_messages", coordinate_cmake)
        self.assertIn("message_runtime", coordinate_cmake)
        self.assertIn(
            "message_generation",
            {element.text for element in coordinate_manifest.findall("build_depend")},
        )
        self.assertIn(
            "message_runtime",
            {element.text for element in coordinate_manifest.findall("exec_depend")},
        )
        self.assertIn("air_ground_coordinate_transform", bringup_cmake)
        self.assertIn(
            "air_ground_coordinate_transform",
            {element.text for element in bringup_manifest.findall("depend")},
        )

    def test_written_rostest_retains_every_transform_listener(self):
        rostest_path = Path(__file__).parent / "test_registration_node.py"
        tree = ast.parse(rostest_path.read_text())
        listener_calls = []
        assigned_calls = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            function = node.func
            if not (
                isinstance(function, ast.Attribute)
                and function.attr == "TransformListener"
            ):
                continue
            listener_calls.append(node)
            parent = next(
                candidate
                for candidate in ast.walk(tree)
                if isinstance(candidate, (ast.Assign, ast.AnnAssign))
                and getattr(candidate, "value", None) is node
            ) if any(
                isinstance(candidate, (ast.Assign, ast.AnnAssign))
                and getattr(candidate, "value", None) is node
                for candidate in ast.walk(tree)
            ) else None
            if parent is not None:
                assigned_calls.append(node)

        self.assertEqual(len(listener_calls), 2)
        self.assertEqual(len(assigned_calls), len(listener_calls))

    def test_timer_ticks_periodic_coordinator_and_publishes_due_decision(self):
        module = load_registration_module(now=11.05)
        due_decision = object()

        class Coordinator:
            def __init__(self):
                self.lock = threading.RLock()
                self.tick_times = []

            def tick(self, now):
                self.tick_times.append(now)
                return due_decision

            @staticmethod
            def snapshot():
                state = SimpleNamespace(initialized=False, revision=0)
                return SimpleNamespace(state=state, status="ACQUIRING_INITIAL")

            @staticmethod
            def complete_publication_cycle():
                pass

        target = module.Registration.__new__(module.Registration)
        target.coordinator = Coordinator()
        target.origin_to_uav_odom = None
        target.registration_mode = "periodic"
        target.status_pub = Publisher("status")
        target.revision_pub = Publisher("revision")
        target.valid_pub = Publisher("valid")
        target.frozen_pub = Publisher("frozen")
        published_decisions = []
        target.publish_decision = published_decisions.append

        target.publish(None)

        self.assertEqual(target.coordinator.tick_times, [11.05])
        self.assertEqual(published_decisions, [due_decision])

    def test_coordinator_odometry_acceptance_is_monotonic_and_finite(self):
        target = make_coordinator()

        accepted = target.observe_odometry("uav", 1.0, 0.0, 0.0)
        duplicate = target.observe_odometry("uav", 1.0, 100.0, 0.0)
        out_of_order = target.observe_odometry("uav", 0.9, 50.0, 0.0)
        nonfinite = target.observe_odometry("uav", 1.1, float("nan"), 0.0)

        self.assertTrue(getattr(accepted, "accepted", False))
        self.assertFalse(getattr(duplicate, "accepted", True))
        self.assertFalse(getattr(out_of_order, "accepted", True))
        self.assertFalse(getattr(nonfinite, "accepted", True))

    def test_uav_callback_rejects_bad_odometry_before_any_buffer_mutation(self):
        module = load_registration_module()
        target = module.Registration.__new__(module.Registration)
        target.uav_input_parent = "expected_parent"
        target.uav_input_child = "expected_child"
        target.coordinator = make_coordinator()
        target.coordinator.add_sample(
            registration_sample([0.0, 0.0, 0.0], 0.9), now=0.9
        )
        target.uav = []
        target.uav_buffer = OdomBuffer(maxlen=20, max_bracket=0.2)
        target.origin_to_uav_odom = object()

        target.uav_callback(odometry_message(1.0, 0.0))
        target.uav_callback(odometry_message(1.0, 100.0))
        target.uav_callback(odometry_message(0.95, 50.0))
        target.uav_callback(odometry_message(1.05, 25.0, z=float("nan")))
        target.uav_callback(odometry_message(1.06, 30.0, frame_id="wrong_parent"))
        target.uav_callback(odometry_message(1.07, 35.0, twist_x=float("nan")))
        target.uav_callback(odometry_message(1.1, 11.0))

        self.assertEqual(len(target.uav), 2)
        interpolated = target.uav_buffer.interpolate_full(Stamp(1.05))
        self.assertIsNotNone(interpolated)
        self.assertAlmostEqual(interpolated[0], 5.5)
        state = target.coordinator.snapshot().state
        self.assertEqual(state.stamp, 1.1)
        np.testing.assert_allclose(
            np.diag(state.covariance),
            [2.220101, 2.220101, 0.2220251],
            rtol=0.0,
            atol=1e-14,
        )

    def test_ugv_callback_rejects_bad_odometry_before_any_buffer_mutation(self):
        module = load_registration_module()
        target = module.Registration.__new__(module.Registration)
        target.ugv_input_parent = "expected_parent"
        target.ugv_input_child = "expected_child"
        target.coordinator = make_coordinator()
        target.coordinator.add_sample(
            registration_sample([0.0, 0.0, 0.0], 0.9), now=0.9
        )
        target.ugv = []
        target.ugv_buffer = OdomBuffer(maxlen=20, max_bracket=0.2)

        target.ugv_callback(odometry_message(1.0, 0.0))
        target.ugv_callback(odometry_message(1.0, 100.0))
        target.ugv_callback(odometry_message(0.95, 50.0))
        target.ugv_callback(odometry_message(1.05, 25.0, z=float("nan")))
        target.ugv_callback(odometry_message(1.06, 30.0, child_frame_id="wrong_child"))
        target.ugv_callback(odometry_message(1.07, 35.0, twist_x=float("nan")))
        target.ugv_callback(odometry_message(1.1, 11.0))

        self.assertEqual(len(target.ugv), 2)
        interpolated = target.ugv_buffer.interpolate_full(Stamp(1.05))
        self.assertIsNotNone(interpolated)
        self.assertAlmostEqual(interpolated[0], 5.5)
        state = target.coordinator.snapshot().state
        self.assertEqual(state.stamp, 1.1)
        np.testing.assert_allclose(
            np.diag(state.covariance),
            [3.320101, 3.320101, 0.3320251],
            rtol=0.0,
            atol=1e-14,
        )

    def test_timer_publishes_updating_snapshot_before_completing_cycle(self):
        module = load_registration_module(now=5.0)

        class Coordinator:
            def __init__(self, status_pub):
                self.lock = threading.RLock()
                self.status_pub = status_pub
                self.completed = 0

            @staticmethod
            def tick(_now):
                return None

            @staticmethod
            def snapshot():
                state = SimpleNamespace(
                    initialized=True,
                    revision=2,
                    mean=np.array([1.0, -2.0, 0.2]),
                    covariance=np.eye(3),
                    stamp=5.0,
                )
                return SimpleNamespace(state=state, status="UPDATING")

            def complete_publication_cycle(self):
                self.assert_updating_was_published()
                self.completed += 1

            def assert_updating_was_published(self):
                if not self.status_pub.messages or self.status_pub.messages[-1] != "UPDATING":
                    raise AssertionError("UPDATING was not published before completion")

        target = module.Registration.__new__(module.Registration)
        target.origin_to_uav_odom = np.eye(4)
        target.registration_mode = "opportunistic"
        target.origin_frame = "origin"
        target.uav_odom_frame = "experimental_uav_odom"
        target.ugv_odom_frame = "experimental_ugv_odom"
        target.status_pub = Publisher("status")
        target.revision_pub = Publisher("revision")
        target.valid_pub = Publisher("valid")
        target.frozen_pub = Publisher("frozen")
        target.ugv = []
        target.send_tf = lambda *_args: None
        target.publish_estimate = lambda *_args: None
        target.coordinator = Coordinator(target.status_pub)

        target.publish(None)

        self.assertEqual(target.status_pub.messages[-1], "UPDATING")
        self.assertEqual(target.coordinator.completed, 1)

    def test_timer_publishes_grown_covariance_with_unchanged_state_revision(self):
        module = load_registration_module(now=6.0)
        states = [
            SimpleNamespace(
                initialized=True,
                revision=4,
                mean=np.array([1.0, -2.0, 0.2]),
                covariance=np.array(
                    [
                        [0.1, 0.01, -0.02],
                        [0.01, 0.2, 0.03],
                        [-0.02, 0.03, 0.3],
                    ]
                ),
                stamp=5.0,
            ),
            SimpleNamespace(
                initialized=True,
                revision=4,
                mean=np.array([1.0, -2.0, 0.2]),
                covariance=np.array(
                    [
                        [0.4, 0.04, -0.05],
                        [0.04, 0.5, 0.06],
                        [-0.05, 0.06, 0.6],
                    ]
                ),
                stamp=6.0,
            ),
        ]

        class Coordinator:
            def __init__(self):
                self.lock = threading.RLock()
                self.snapshot_index = 0

            @staticmethod
            def tick(_now):
                return None

            def snapshot(self):
                state = states[self.snapshot_index]
                self.snapshot_index += 1
                return SimpleNamespace(state=state, status="TRACKING")

            @staticmethod
            def complete_publication_cycle():
                pass

        target = module.Registration.__new__(module.Registration)
        target.coordinator = Coordinator()
        target.origin_to_uav_odom = np.eye(4)
        target.registration_mode = "opportunistic"
        target.origin_frame = "origin"
        target.uav_odom_frame = "uav_odom"
        target.ugv_odom_frame = "ugv_odom"
        target.status_pub = Publisher("status")
        target.revision_pub = Publisher("revision")
        target.valid_pub = Publisher("valid")
        target.frozen_pub = Publisher("frozen")
        target.estimate_pub = Publisher("estimate")
        target.state_pub = Publisher("state")
        target.ugv = []
        target.send_tf = lambda *_args: None

        target.publish(None)
        target.publish(None)

        expected_pose = (
            1.0,
            -2.0,
            0.0,
            0.0,
            0.0,
            0.09983341664682815,
            0.9950041652780258,
        )
        expected_covariances = [
            [
                0.1, 0.01, 0.0, 0.0, 0.0, -0.02,
                0.01, 0.2, 0.0, 0.0, 0.0, 0.03,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                -0.02, 0.03, 0.0, 0.0, 0.0, 0.3,
            ],
            [
                0.4, 0.04, 0.0, 0.0, 0.0, -0.05,
                0.04, 0.5, 0.0, 0.0, 0.0, 0.06,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                -0.05, 0.06, 0.0, 0.0, 0.0, 0.6,
            ],
        ]
        expected_planar_covariances = [
            (0.1, 0.01, -0.02, 0.01, 0.2, 0.03, -0.02, 0.03, 0.3),
            (0.4, 0.04, -0.05, 0.04, 0.5, 0.06, -0.05, 0.06, 0.6),
        ]

        self.assertEqual(len(target.estimate_pub.messages), 2)
        self.assertEqual(len(target.state_pub.messages), 2)
        for index, (estimate, continuous) in enumerate(
            zip(target.estimate_pub.messages, target.state_pub.messages)
        ):
            self.assertEqual(continuous.revision, 4)
            self.assertEqual(estimate.header.stamp.to_sec(), [5.0, 6.0][index])
            self.assertEqual(continuous.header.stamp.to_sec(), [5.0, 6.0][index])
            self.assertEqual(estimate.header.frame_id, "origin")
            self.assertEqual(continuous.header.frame_id, "origin")
            self.assertEqual(pose_components(estimate.pose), pose_components(continuous.pose))
            for actual, expected in zip(pose_components(continuous.pose), expected_pose):
                self.assertAlmostEqual(actual, expected)
            self.assertEqual(estimate.pose.covariance, expected_covariances[index])
            self.assertEqual(continuous.pose.covariance, expected_covariances[index])
            for message in (estimate.pose, continuous.pose):
                self.assertEqual(
                    tuple(
                        message.covariance[covariance_index]
                        for covariance_index in PLANAR_COVARIANCE_INDICES
                    ),
                    expected_planar_covariances[index],
                )

    def test_periodic_due_event_uses_one_tf_estimate_revision_publication_path(self):
        module = load_registration_module(now=5.0)
        state = SimpleNamespace(
            initialized=True,
            revision=2,
            mean=np.array([1.0, -2.0, 0.2]),
            covariance=np.eye(3),
            stamp=5.0,
        )
        decision = SimpleNamespace(
            accepted=True,
            revision=2,
            reason="accepted",
            status="UPDATING",
            inlier_count=20,
            mahalanobis=1.0,
            state=state,
        )

        class Coordinator:
            def __init__(self):
                self.lock = threading.RLock()
                self.completed = 0

            @staticmethod
            def tick(_now):
                return decision

            @staticmethod
            def snapshot():
                return SimpleNamespace(state=state, status="UPDATING")

            def complete_publication_cycle(self):
                self.completed += 1

        target = module.Registration.__new__(module.Registration)
        target.coordinator = Coordinator()
        target.origin_to_uav_odom = np.eye(4)
        target.registration_mode = "periodic"
        target.origin_frame = "origin"
        target.uav_odom_frame = "experimental_uav_odom"
        target.ugv_odom_frame = "experimental_ugv_odom"
        target.status_pub = Publisher("status")
        target.count_pub = Publisher("count")
        target.valid_pub = Publisher("valid")
        target.frozen_pub = Publisher("frozen")
        target.innovation_pub = Publisher("innovation")
        target.revision_pub = Publisher("revision")
        target.accepted_update_pub = Publisher("accepted_update")
        target.ugv = []
        sent_tf = []
        estimates = []
        target.send_tf = lambda parent, child, matrix, stamp: sent_tf.append(
            (parent, child, matrix.copy(), stamp.to_sec())
        )
        def publish_estimate(value, revision=None):
            estimates.append((value, revision))
            message = PoseWithCovarianceStamped()
            message.header.stamp = Stamp(value.stamp)
            message.header.frame_id = target.origin_frame
            return message

        target.publish_estimate = publish_estimate

        target.publish(None)

        ugv_tf = [item for item in sent_tf if item[1] == "experimental_ugv_odom"]
        self.assertEqual(len(ugv_tf), 1)
        self.assertEqual(len(estimates), 1)
        self.assertEqual(target.revision_pub.messages, [2])
        self.assertEqual(target.coordinator.completed, 1)

    def test_accepted_event_publishes_new_tf_and_estimate_before_revision(self):
        module = load_registration_module(now=5.0)
        events = []
        boundary = {"tf_x": None, "estimate_revision": None}
        target = module.Registration.__new__(module.Registration)
        target.origin_frame = "origin"
        target.ugv_odom_frame = "experimental_ugv_odom"
        target.status_pub = Publisher("status", events)
        target.count_pub = Publisher("count", events)
        target.valid_pub = Publisher("valid", events)
        target.frozen_pub = Publisher("frozen", events)
        target.innovation_pub = Publisher("innovation", events)
        target.accepted_update_pub = Publisher("accepted_update", events)

        def revision_boundary(message):
            self.assertEqual(message, 2)
            self.assertEqual(boundary["tf_x"], 4.0)
            self.assertEqual(boundary["estimate_revision"], 2)

        target.revision_pub = Publisher("revision", events, revision_boundary)

        def send_tf(_parent, _child, matrix, _stamp):
            boundary["tf_x"] = matrix[0, 2]
            events.append("tf")

        def publish_estimate(state, revision=None):
            boundary["estimate_revision"] = revision
            events.append("estimate")
            message = PoseWithCovarianceStamped()
            message.header.stamp = Stamp(state.stamp)
            message.header.frame_id = target.origin_frame
            return message

        target.send_tf = send_tf
        target.publish_estimate = publish_estimate
        state = SimpleNamespace(
            mean=np.array([4.0, -3.0, 0.2]),
            covariance=np.eye(3),
            stamp=5.0,
        )
        decision = SimpleNamespace(
            accepted=True,
            revision=2,
            reason="accepted",
            status="UPDATING",
            inlier_count=20,
            mahalanobis=1.0,
            state=state,
        )

        target.publish_decision(decision)

        self.assertLess(events.index("tf"), events.index("revision"))
        self.assertLess(events.index("estimate"), events.index("revision"))

    def test_accepted_event_publishes_matching_independent_state_before_event(self):
        module = load_registration_module()
        events = []
        target = module.Registration.__new__(module.Registration)
        target.origin_frame = "origin"
        target.ugv_odom_frame = "ugv_odom"
        target.status_pub = Publisher("status", events)
        target.count_pub = Publisher("count", events)
        target.valid_pub = Publisher("valid", events)
        target.frozen_pub = Publisher("frozen", events)
        target.innovation_pub = Publisher("innovation", events)
        target.estimate_pub = Publisher("estimate", events)
        target.state_pub = Publisher("state", events)
        target.accepted_update_pub = Publisher("accepted_update", events)
        target.revision_pub = Publisher("revision", events)
        target.send_tf = lambda *_args: events.append("tf")
        covariance = np.array(
            [
                [0.1, 0.01, -0.02],
                [0.01, 0.2, 0.03],
                [-0.02, 0.03, 0.4],
            ]
        )
        state = SimpleNamespace(
            mean=np.array([1.0, -2.0, 0.3]),
            covariance=covariance,
            revision=3,
            stamp=4.0,
        )
        decision = SimpleNamespace(
            accepted=True,
            revision=3,
            status="UPDATING",
            inlier_count=20,
            mahalanobis=1.0,
            state=state,
        )

        target.publish_decision(decision)

        self.assertEqual(len(target.estimate_pub.messages), 1)
        self.assertEqual(len(target.state_pub.messages), 1)
        self.assertEqual(len(target.accepted_update_pub.messages), 1)
        estimate = target.estimate_pub.messages[0]
        continuous = target.state_pub.messages[0]
        accepted = target.accepted_update_pub.messages[0]
        expected_covariance = [
            0.1, 0.01, 0.0, 0.0, 0.0, -0.02,
            0.01, 0.2, 0.0, 0.0, 0.0, 0.03,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            -0.02, 0.03, 0.0, 0.0, 0.0, 0.4,
        ]
        expected_planar_covariance = (
            0.1, 0.01, -0.02, 0.01, 0.2, 0.03, -0.02, 0.03, 0.4
        )
        expected_pose = (
            1.0,
            -2.0,
            0.0,
            0.0,
            0.0,
            0.14943813247359922,
            0.9887710779360422,
        )

        self.assertEqual(continuous.revision, 3)
        self.assertEqual(accepted.revision, 3)
        self.assertEqual(estimate.header.stamp.to_sec(), 4.0)
        self.assertEqual(continuous.header.stamp.to_sec(), 4.0)
        self.assertEqual(accepted.header.stamp.to_sec(), 4.0)
        self.assertEqual(estimate.header.frame_id, "origin")
        self.assertEqual(continuous.header.frame_id, "origin")
        self.assertEqual(accepted.header.frame_id, "origin")
        for message in (estimate.pose, continuous.pose, accepted.pose):
            for actual, expected in zip(pose_components(message), expected_pose):
                self.assertAlmostEqual(actual, expected)
            self.assertEqual(message.covariance, expected_covariance)
            self.assertEqual(
                tuple(
                    message.covariance[index]
                    for index in PLANAR_COVARIANCE_INDICES
                ),
                expected_planar_covariance,
            )
        self.assertEqual(pose_components(estimate.pose), pose_components(continuous.pose))
        self.assertEqual(pose_components(estimate.pose), pose_components(accepted.pose))
        self.assertIsNot(estimate.pose, continuous.pose)
        self.assertIsNot(estimate.pose, accepted.pose)
        self.assertIsNot(continuous.pose, accepted.pose)
        continuous.pose.pose.position.x = 99.0
        continuous.pose.covariance[0] = 99.0
        self.assertEqual(estimate.pose.pose.position.x, 1.0)
        self.assertEqual(accepted.pose.pose.position.x, 1.0)
        self.assertEqual(estimate.pose.covariance[0], 0.1)
        self.assertEqual(accepted.pose.covariance[0], 0.1)
        self.assertLess(events.index("estimate"), events.index("state"))
        self.assertLess(events.index("state"), events.index("accepted_update"))
        self.assertLess(events.index("accepted_update"), events.index("revision"))
        self.assertLess(events.index("revision"), events.index("status"))


if __name__ == "__main__":
    unittest.main()
