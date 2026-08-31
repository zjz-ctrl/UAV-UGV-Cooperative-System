#!/usr/bin/env python3
"""Script-level adapter tests using duck-typed ROS messages and fake rospy."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parents[1] / "src"))

import ros_stubs
from ros_stubs import (
    FakeTime,
    Header,
    Odometry,
    PositionCommand,
    Quaternion,
    Vector3,
    load_script_class,
)


def make_odometry(stamp, frame_id, child_frame_id):
    message = Odometry()
    message.header = Header(stamp=FakeTime(stamp), frame_id=frame_id)
    message.child_frame_id = child_frame_id
    message.pose.pose.position.x = 1.0
    message.pose.pose.orientation.w = 1.0
    return message


class OdomPerturbationNodeTest(unittest.TestCase):
    def setUp(self):
        self.rospy, self.saved = ros_stubs.install_fake_ros()
        node_class = load_script_class(
            "odom_perturbation_node.py", "OdomPerturbationNode"
        )
        self.rospy.parameters = {
            "~source_topic": "/src/odom",
            "~destination_topic": "/experiment/odom",
            "~source_frame": "map",
            "~destination_frame": "experiment_uav",
            "~initial_xyyaw": [0.0, 0.0, 0.0],
            "~truth_topic": "/truth/frame",
            "~epoch_seconds": 1000.0,
        }
        self.node = node_class()

    def tearDown(self):
        ros_stubs.restore_ros(self.saved)

    def callback(self):
        return self.rospy.subscribers[0].callback

    def truth_publications(self):
        return self.node.truth_publisher.published

    def test_stamp_after_epoch_transforms_from_shared_epoch(self):
        self.callback()(make_odometry(1000.0, "map", "base_link"))
        self.callback()(make_odometry(1003.5, "map", "base_link"))

        self.assertEqual(len(self.truth_publications()), 2)
        first_truth = eval(self.truth_publications()[0].data)
        second_truth = eval(self.truth_publications()[1].data)
        self.assertEqual(first_truth["stamp"], 1000.0)
        self.assertEqual(second_truth["stamp"], 1003.5)

    def test_stamp_before_epoch_fails_fast_without_publishing(self):
        self.callback()(make_odometry(999.0, "map", "base_link"))

        self.assertEqual(len(self.node.odom_publisher.published), 0)
        self.assertEqual(len(self.truth_publications()), 0)

    def test_extreme_elapsed_time_is_rejected_not_materialized(self):
        self.node.perturbation.maximum_elapsed_seconds = 60.0

        self.callback()(make_odometry(1000.0 + 1.8e9, "map", "base_link"))

        self.assertEqual(len(self.node.odom_publisher.published), 0)


class PositionCommandAdapterTest(unittest.TestCase):
    def setUp(self):
        self.rospy, self.saved = ros_stubs.install_fake_ros()
        node_class = load_script_class(
            "position_command_adapter.py", "PositionCommandAdapter"
        )
        self.rospy.parameters = {
            "~initial_xyyaw": [0.0, 0.0, 0.0],
            "~destination_topic": "/iris_0/position_cmd",
            "~destination_frame": "iris_0/odom",
            "~epoch_seconds": 500.0,
        }
        self.node = node_class()

    def tearDown(self):
        ros_stubs.restore_ros(self.saved)

    def make_command(self, stamp):
        command = PositionCommand()
        command.header = Header(stamp=FakeTime(stamp), frame_id="experiment")
        command.position.x = 5.0
        command.yaw = 0.1
        command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return command

    def test_command_stamp_before_epoch_is_dropped(self):
        self.node.callback(self.make_command(499.0))

        self.assertEqual(len(self.node.publisher.published), 0)

    def test_command_after_epoch_is_inverted_and_published(self):
        self.node.callback(self.make_command(502.0))

        self.assertEqual(len(self.node.publisher.published), 1)
        output = self.node.publisher.published[0]
        self.assertEqual(output.header.frame_id, "iris_0/odom")
        self.assertAlmostEqual(output.position.x, 5.0)
        self.assertAlmostEqual(output.yaw, 0.1)


class DomainSeedPlumbingTest(unittest.TestCase):
    def setUp(self):
        self.rospy, self.saved = ros_stubs.install_fake_ros()
        node_class = load_script_class(
            "odom_perturbation_node.py", "OdomPerturbationNode"
        )

        def build(domain):
            self.rospy.parameters = {
                "~source_topic": "/src/odom",
                "~destination_topic": "/experiment/odom",
                "~source_frame": "map",
                "~destination_frame": "experiment",
                "~truth_topic": "/truth/frame",
                "~seed": 17,
                "~seed_domain": domain,
                "~epoch_seconds": 0.0,
            }
            return node_class()

        self.build = build

    def tearDown(self):
        ros_stubs.restore_ros(self.saved)

    def test_node_walks_use_domain_separated_effective_seeds(self):
        from air_ground_experiments.frame_perturbation import domain_seed

        uav_node = self.build("uav")
        ugv_node = self.build("ugv")

        self.assertEqual(uav_node.perturbation.seed, domain_seed(17, "uav"))
        self.assertEqual(ugv_node.perturbation.seed, domain_seed(17, "ugv"))
        self.assertNotEqual(
            uav_node.perturbation.seed, ugv_node.perturbation.seed
        )

    def test_truth_message_keeps_trial_seed_alongside_stream_seed(self):
        import json as json_module
        from air_ground_experiments.frame_perturbation import domain_seed

        node = self.build("uav")
        message = make_odometry(1.5, "map", "base_link")
        self.rospy.subscribers[0].callback(message)

        payload = json_module.loads(node.truth_publisher.published[0].data)
        self.assertEqual(payload["trial_seed"], 17)
        self.assertEqual(payload["seed"], domain_seed(17, "uav"))


class CommandAdapterSeedParityTest(unittest.TestCase):
    def setUp(self):
        self.rospy, self.saved = ros_stubs.install_fake_ros()
        node_class = load_script_class(
            "position_command_adapter.py", "PositionCommandAdapter"
        )
        self.rospy.parameters = {
            "~destination_topic": "/iris_0/position_cmd",
            "~destination_frame": "iris_0/odom",
            "~seed": 17,
            "~seed_domain": "uav",
            "~epoch_seconds": 0.0,
            "~translational_drift_rate": 0.05,
            "~yaw_drift_rate": 0.01,
        }
        self.node = node_class()

    def tearDown(self):
        ros_stubs.restore_ros(self.saved)

    def test_adapter_stream_matches_uav_odom_domain_seed(self):
        from air_ground_experiments.frame_perturbation import domain_seed

        self.assertEqual(
            self.node.perturbation.seed, domain_seed(17, "uav")
        )


class OdomTwistConventionPlumbingTest(unittest.TestCase):
    def build(self, convention, stamp=10.0, drift=0.0):
        self.rospy, self.saved = ros_stubs.install_fake_ros()
        node_class = load_script_class(
            "odom_perturbation_node.py", "OdomPerturbationNode"
        )
        self.rospy.parameters = {
            "~source_topic": "/src/odom",
            "~destination_topic": "/experiment/odom",
            "~source_frame": "map",
            "~destination_frame": "experiment",
            "~truth_topic": "/truth/frame",
            "~seed": 3,
            "~seed_domain": "uav",
            "~epoch_seconds": 0.0,
            "~translational_drift_rate": drift,
            "~yaw_drift_rate": drift,
            "~twist_convention": convention,
        }
        node = node_class()
        message = make_odometry(stamp, "map", "base_link")
        message.twist.twist.linear.x = 1.0
        self.rospy.subscribers[0].callback(message)
        return node.odom_publisher.published[0]

    def tearDown(self):
        ros_stubs.restore_ros(self.saved)

    def test_parent_convention_relabels_child_and_rotates_linear_x(self):
        output = self.build("parent")

        self.assertEqual(output.child_frame_id, "experiment")

    def test_body_convention_preserves_child_label(self):
        output = self.build("body")

        self.assertEqual(output.child_frame_id, "base_link")


class ObservationGateNodeTest(unittest.TestCase):
    def setUp(self):
        self.rospy, self.saved = ros_stubs.install_fake_ros()
        node_class = load_script_class("observation_gate.py", "ObservationGateNode")
        self.node_class = node_class
        self.rospy.parameters = {
            "~visibility_windows": [[0.0, 5.0]],
            "~delay_seconds": 0.4,
            "~delay_jitter_seconds": 0.0,
            "~outlier_probability": 0.0,
            "~outlier_translation_m": 0.0,
            "~outlier_yaw_rad": 0.0,
            "~seed": 17,
            "~seed_domain": "gate",
            "~epoch_seconds": 1000.0,
            "~source_topic": "/detector/observation",
            "~destination_topic": "/experiment/observation",
            "~diagnostic_topic": "/experiment/diagnostic",
        }

    def tearDown(self):
        ros_stubs.restore_ros(self.saved)

    def observation(self, stamp):
        message = ros_stubs.PoseWithCovarianceStamped()
        message.header = Header(stamp=FakeTime(stamp), frame_id="camera")
        return message

    def test_gate_stream_uses_gate_domain_seed(self):
        from air_ground_experiments.frame_perturbation import domain_seed

        node = self.node_class()

        self.assertEqual(node.schedule.seed, domain_seed(17, "gate"))

    def test_callback_visibility_uses_epoch_relative_image_stamp(self):
        import json as json_module
        from air_ground_experiments.frame_perturbation import domain_seed

        node = self.node_class()
        callback = self.rospy.subscribers[0].callback
        timer = self.rospy.timers[0].callback

        self.rospy.now_seconds = 1004.9
        callback(self.observation(1003.5))
        self.rospy.now_seconds = 1012.0
        timer(None)

        self.assertEqual(len(node.publisher.published), 1)
        published = node.publisher.published[0]
        self.assertEqual(published.header.stamp.to_sec(), 1003.5)
        diagnostic = json_module.loads(node.diagnostic_publisher.published[0].data)
        self.assertEqual(diagnostic["image_stamp"], 1003.5)
        self.assertEqual(diagnostic["receipt_time"], 1004.9)
        self.assertEqual(diagnostic["actual_release"], 1012.0)
        self.assertAlmostEqual(diagnostic["scheduled_release"], 1005.3)
        self.assertEqual(diagnostic["trial_seed"], 17)
        self.assertEqual(diagnostic["seed"], domain_seed(17, "gate"))

    def test_hidden_observation_is_dropped_without_synthesis(self):
        node = self.node_class()
        callback = self.rospy.subscribers[0].callback
        timer = self.rospy.timers[0].callback

        self.rospy.now_seconds = 1006.0
        callback(self.observation(1006.0))
        self.rospy.now_seconds = 1020.0
        timer(None)

        self.assertEqual(len(node.publisher.published), 0)
        self.assertEqual(len(node.diagnostic_publisher.published), 0)


class ObservationGateControlTest(unittest.TestCase):
    """Runtime scenario control: pass/hide/outlier commands on the input gate."""

    def setUp(self):
        self.rospy, self.saved = ros_stubs.install_fake_ros()
        node_class = load_script_class("observation_gate.py", "ObservationGateNode")
        self.node_class = node_class
        self.rospy.parameters = {
            "~visibility_windows": [[0.0, 1.0e9]],
            "~delay_seconds": 0.0,
            "~delay_jitter_seconds": 0.0,
            "~outlier_probability": 0.0,
            "~outlier_translation_m": 0.0,
            "~outlier_yaw_rad": 0.0,
            "~seed": 17,
            "~seed_domain": "gate",
            "~epoch_seconds": 1000.0,
            "~source_topic": "/detector/observation",
            "~destination_topic": "/experiment/observation",
            "~diagnostic_topic": "/experiment/diagnostic",
            "~control_topic": "/experiment/control",
        }

    def tearDown(self):
        ros_stubs.restore_ros(self.saved)

    def observation(self, stamp):
        message = ros_stubs.PoseWithCovarianceStamped()
        message.header = Header(stamp=FakeTime(stamp), frame_id="camera")
        message.pose.pose.position.x = 0.3
        message.pose.pose.orientation.w = 1.0
        return message

    def control_callback(self):
        self.assertEqual(len(self.rospy.subscribers), 2)
        return self.rospy.subscribers[1].callback

    def test_control_subscription_is_optional_and_defaults_to_pass(self):
        del self.rospy.parameters["~control_topic"]
        node = self.node_class()

        self.assertEqual(len(self.rospy.subscribers), 1)
        self.assertEqual(node.control_mode, "pass")

    def test_invalid_command_is_ignored_without_mode_change(self):
        self.node_class()
        control = self.control_callback()

        control(ros_stubs.String(data="explode"))

        self.assertEqual(self.rospy.subscribers[0] is not None, True)

    def test_hide_command_drops_observations_until_pass_restores(self):
        node = self.node_class()
        callback = self.rospy.subscribers[0].callback
        control = self.control_callback()
        timer = self.rospy.timers[0].callback

        control(ros_stubs.String(data="hide"))
        self.rospy.now_seconds = 1006.0
        callback(self.observation(1006.0))
        self.rospy.now_seconds = 1007.0
        timer(None)
        self.assertEqual(len(node.publisher.published), 0)

        control(ros_stubs.String(data="pass"))
        callback(self.observation(1008.0))
        self.rospy.now_seconds = 1009.0
        timer(None)
        self.assertEqual(len(node.publisher.published), 1)

    def test_outlier_command_applies_manual_gross_outlier(self):
        import json as json_module

        node = self.node_class()
        callback = self.rospy.subscribers[0].callback
        control = self.control_callback()
        timer = self.rospy.timers[0].callback

        control(ros_stubs.String(data="outlier 1.5 -0.8 0.0"))
        self.rospy.now_seconds = 1006.0
        callback(self.observation(1006.0))
        self.rospy.now_seconds = 1007.0
        timer(None)

        self.assertEqual(len(node.publisher.published), 1)
        published = node.publisher.published[0]
        self.assertAlmostEqual(published.pose.pose.position.x, 1.8)
        self.assertAlmostEqual(published.pose.pose.position.y, -0.8)
        diagnostic = json_module.loads(node.diagnostic_publisher.published[0].data)
        self.assertEqual(diagnostic["outlier_xyyaw"], [1.5, -0.8, 0.0])

    def test_pass_command_clears_manual_outlier(self):
        node = self.node_class()
        callback = self.rospy.subscribers[0].callback
        control = self.control_callback()
        timer = self.rospy.timers[0].callback

        control(ros_stubs.String(data="outlier 1.5 -0.8 0.0"))
        control(ros_stubs.String(data="pass"))
        self.rospy.now_seconds = 1006.0
        callback(self.observation(1006.0))
        self.rospy.now_seconds = 1007.0
        timer(None)

        published = node.publisher.published[0]
        self.assertAlmostEqual(published.pose.pose.position.x, 0.3)
        self.assertAlmostEqual(published.pose.pose.position.y, 0.0)


if __name__ == "__main__":
    unittest.main()
