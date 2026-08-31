#!/usr/bin/env python3

import math
import time
import unittest

import numpy as np
import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String, UInt32
from tf.transformations import (
    concatenate_matrices,
    euler_from_quaternion,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    quaternion_slerp,
    translation_matrix,
)


class RegistrationNodeTest(unittest.TestCase):
    UAV_PARENT = "test_uav_input_odom"
    UAV_CHILD = "test_uav_input_base"
    UGV_PARENT = "test_ugv_input_odom"
    UGV_CHILD = "test_ugv_input_base"
    VALIDATION_CASES = ("ugv_validation", "observation_validation")

    @classmethod
    def setUpClass(cls):
        rospy.init_node("registration_node_test")
        cls.uav_pub = rospy.Publisher(
            "/test/registration/uav_odom", Odometry, queue_size=20
        )
        cls.ugv_pub = rospy.Publisher(
            "/test/registration/ugv_odom", Odometry, queue_size=20
        )
        cls.observation_pub = rospy.Publisher(
            "/test/registration/observation",
            PoseWithCovarianceStamped,
            queue_size=20,
        )
        cls.repeated_uav_pub = rospy.Publisher(
            "/test/repeated/uav_odom", Odometry, queue_size=20
        )
        cls.repeated_ugv_pub = rospy.Publisher(
            "/test/repeated/ugv_odom", Odometry, queue_size=20
        )
        cls.repeated_observation_pub = rospy.Publisher(
            "/test/repeated/observation",
            PoseWithCovarianceStamped,
            queue_size=20,
        )
        cls.validation_pubs = {}
        for case in cls.VALIDATION_CASES:
            prefix = "/test/{}/".format(case)
            cls.validation_pubs[case] = (
                rospy.Publisher(prefix + "uav_odom", Odometry, queue_size=20),
                rospy.Publisher(prefix + "ugv_odom", Odometry, queue_size=20),
                rospy.Publisher(
                    prefix + "observation",
                    PoseWithCovarianceStamped,
                    queue_size=20,
                ),
            )
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            publishers = [
                cls.uav_pub,
                cls.ugv_pub,
                cls.observation_pub,
                cls.repeated_uav_pub,
                cls.repeated_ugv_pub,
                cls.repeated_observation_pub,
            ]
            publishers.extend(
                publisher
                for case_publishers in cls.validation_pubs.values()
                for publisher in case_publishers
            )
            if all(publisher.get_num_connections() for publisher in publishers):
                return
            rospy.sleep(0.05)
        raise RuntimeError("registration node subscribers did not connect")

    @staticmethod
    def make_odom(
        stamp,
        frame_id,
        child_frame_id,
        x=0.0,
        y=0.0,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    ):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.child_frame_id = child_frame_id
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = z
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = quaternion
        return message

    @staticmethod
    def make_observation(stamp, matrix, frame_id="test_camera"):
        message = PoseWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.pose.pose.position.x = matrix[0, 3]
        message.pose.pose.position.y = matrix[1, 3]
        message.pose.pose.position.z = matrix[2, 3]
        quaternion = quaternion_from_matrix(matrix)
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = quaternion
        planar_indices = (0, 1, 5)
        planar_covariance = (
            (4e-6, 1e-6, 5e-7),
            (1e-6, 9e-6, -4e-7),
            (5e-7, -4e-7, 1e-6),
        )
        for row, target_row in enumerate(planar_indices):
            for column, target_column in enumerate(planar_indices):
                message.pose.covariance[target_row * 6 + target_column] = (
                    planar_covariance[row][column]
                )
        message.pose.covariance[14] = 2e-6
        message.pose.covariance[21] = 2e-6
        message.pose.covariance[28] = 2e-6
        return message

    def publish_odometry_pair(self, stamp):
        self.uav_pub.publish(
            self.make_odom(stamp, self.UAV_PARENT, self.UAV_CHILD, z=2.0)
        )
        self.ugv_pub.publish(
            self.make_odom(stamp, self.UGV_PARENT, self.UGV_CHILD)
        )
        rospy.sleep(0.03)

    @staticmethod
    def wait_for_status(topic, expected, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.01, deadline - time.monotonic())
            message = rospy.wait_for_message(
                topic, String, timeout=min(0.5, remaining)
            )
            if message.data == expected:
                return message
        raise AssertionError("{} did not reach {}".format(topic, expected))

    @staticmethod
    def wait_for_revision(topic, expected, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.01, deadline - time.monotonic())
            message = rospy.wait_for_message(
                topic, UInt32, timeout=min(0.5, remaining)
            )
            if message.data == expected:
                return message
        raise AssertionError("{} did not reach revision {}".format(topic, expected))

    def capture_validation_origin(self, case, start):
        uav_pub, ugv_pub, _ = self.validation_pubs[case]
        for index in range(3):
            stamp = start + rospy.Duration(0.05 * index)
            uav_pub.publish(
                self.make_odom(stamp, self.UAV_PARENT, self.UAV_CHILD, z=2.0)
            )
            ugv_pub.publish(
                self.make_odom(stamp, self.UGV_PARENT, self.UGV_CHILD)
            )
            rospy.sleep(0.03)
        self.wait_for_status(
            "/test/{}/status".format(case), "ACQUIRING_REGISTRATION"
        )

    def assert_validation_case_does_not_freeze(self, case):
        rospy.sleep(0.3)
        prefix = "/test/{}/".format(case)
        self.assertEqual(
            rospy.wait_for_message(prefix + "status", String, timeout=2.0).data,
            "ACQUIRING_REGISTRATION",
        )
        self.assertEqual(
            rospy.wait_for_message(prefix + "revision", UInt32, timeout=2.0).data,
            0,
        )

    @staticmethod
    def pose_matrix(position, quaternion):
        return concatenate_matrices(translation_matrix(position), quaternion_matrix(quaternion))

    def publish_interpolated_sample(
        self,
        center_stamp,
        desired,
        uav_pub=None,
        ugv_pub=None,
        observation_pub=None,
    ):
        uav_pub = self.uav_pub if uav_pub is None else uav_pub
        ugv_pub = self.ugv_pub if ugv_pub is None else ugv_pub
        observation_pub = (
            self.observation_pub if observation_pub is None else observation_pub
        )
        before = center_stamp - rospy.Duration(0.04)
        after = center_stamp + rospy.Duration(0.04)
        uav_before_position = (0.1, -0.1, 2.0)
        uav_after_position = (0.3, 0.1, 2.2)
        uav_before_quaternion = quaternion_from_euler(0.15, -0.1, 0.05)
        uav_after_quaternion = quaternion_from_euler(0.25, -0.2, 0.15)
        ugv_before_position = (0.4, -0.3, 0.1)
        ugv_after_position = (0.6, -0.1, 0.1)
        ugv_before_quaternion = quaternion_from_euler(0.0, 0.0, 0.1)
        ugv_after_quaternion = quaternion_from_euler(0.0, 0.0, 0.2)
        for stamp, uav_position, uav_quaternion, ugv_position, ugv_quaternion in (
            (before, uav_before_position, uav_before_quaternion, ugv_before_position, ugv_before_quaternion),
            (after, uav_after_position, uav_after_quaternion, ugv_after_position, ugv_after_quaternion),
        ):
            uav_rpy = euler_from_quaternion(uav_quaternion)
            ugv_rpy = euler_from_quaternion(ugv_quaternion)
            uav_pub.publish(
                self.make_odom(
                    stamp,
                    self.UAV_PARENT,
                    self.UAV_CHILD,
                    *uav_position,
                    roll=uav_rpy[0],
                    pitch=uav_rpy[1],
                    yaw=uav_rpy[2],
                )
            )
            ugv_pub.publish(
                self.make_odom(
                    stamp,
                    self.UGV_PARENT,
                    self.UGV_CHILD,
                    *ugv_position,
                    roll=ugv_rpy[0],
                    pitch=ugv_rpy[1],
                    yaw=ugv_rpy[2],
                )
            )
        rospy.sleep(0.04)
        uav = self.pose_matrix(
            (0.2, 0.0, 2.1),
            quaternion_slerp(uav_before_quaternion, uav_after_quaternion, 0.5),
        )
        ugv = self.pose_matrix(
            (0.5, -0.2, 0.1),
            quaternion_slerp(ugv_before_quaternion, ugv_after_quaternion, 0.5),
        )
        origin = translation_matrix((0.0, 0.0, -2.0))
        base_camera = self.pose_matrix(
            (0.0, 0.0, -0.17),
            quaternion_from_euler(math.pi, 0.0, -math.pi / 2.0),
        )
        base_board = self.pose_matrix(
            (-0.3, -0.2, 0.1), quaternion_from_euler(0.0, 0.0, 0.0)
        )
        desired_matrix = self.pose_matrix(
            (desired[0], desired[1], 0.0),
            quaternion_from_euler(0.0, 0.0, desired[2]),
        )
        observation = inverse_matrix(origin.dot(uav).dot(base_camera)).dot(
            desired_matrix
        ).dot(ugv.dot(base_board))
        observation_pub.publish(self.make_observation(center_stamp, observation))
        rospy.sleep(0.05)

    def publish_repeated_window(self, start, desired):
        for index, offset in enumerate((-0.006, 0.004, -0.002, 0.005)):
            self.publish_interpolated_sample(
                start + rospy.Duration(0.3 * index),
                (desired[0] + offset, desired[1] - offset, desired[2] + offset * 0.2),
                self.repeated_uav_pub,
                self.repeated_ugv_pub,
                self.repeated_observation_pub,
            )

    def test_ugv_validator_rejects_a_complete_candidate_batch(self):
        case = "ugv_validation"
        start = rospy.Time.now()
        self.capture_validation_origin(case, start)
        uav_pub, ugv_pub, observation_pub = self.validation_pubs[case]
        invalid_frames = (
            ("", self.UGV_CHILD),
            ("wrong_ugv_parent", self.UGV_CHILD),
            (self.UGV_PARENT, "wrong_ugv_child"),
            ("wrong_ugv_parent", "wrong_ugv_child"),
        )
        identity_observation = translation_matrix((0.0, 0.0, 0.0))
        for index, ugv_frames in enumerate(invalid_frames):
            stamp = start + rospy.Duration(0.4 + 0.3 * index)
            uav_pub.publish(
                self.make_odom(stamp, self.UAV_PARENT, self.UAV_CHILD, z=2.0)
            )
            ugv_pub.publish(self.make_odom(stamp, *ugv_frames))
            rospy.sleep(0.03)
            observation_pub.publish(
                self.make_observation(stamp, identity_observation)
            )
            rospy.sleep(0.03)
        self.assert_validation_case_does_not_freeze(case)

    def test_observation_validator_rejects_a_complete_candidate_batch(self):
        case = "observation_validation"
        start = rospy.Time.now()
        self.capture_validation_origin(case, start)
        uav_pub, ugv_pub, observation_pub = self.validation_pubs[case]
        invalid_frames = ("", "wrong_camera", "", "wrong_camera")
        identity_observation = translation_matrix((0.0, 0.0, 0.0))
        for index, frame_id in enumerate(invalid_frames):
            stamp = start + rospy.Duration(0.4 + 0.3 * index)
            uav_pub.publish(
                self.make_odom(stamp, self.UAV_PARENT, self.UAV_CHILD, z=2.0)
            )
            ugv_pub.publish(
                self.make_odom(stamp, self.UGV_PARENT, self.UGV_CHILD)
            )
            rospy.sleep(0.03)
            observation_pub.publish(
                self.make_observation(
                    stamp, identity_observation, frame_id=frame_id
                )
            )
            rospy.sleep(0.03)
        self.assert_validation_case_does_not_freeze(case)

    def test_one_shot_registration_freezes_once(self):
        initial = rospy.wait_for_message(
            "/air_ground/registration/status", String, timeout=5.0
        )
        self.assertEqual(initial.data, "CAPTURING_ORIGIN")
        self.assertEqual(
            rospy.wait_for_message(
                "/air_ground/registration/revision", UInt32, timeout=2.0
            ).data,
            0,
        )

        start = rospy.Time.now()
        for index, frames in enumerate(
            [
                (("", self.UAV_CHILD), (self.UGV_PARENT, self.UGV_CHILD)),
                (("wrong_uav_parent", self.UAV_CHILD), (self.UGV_PARENT, self.UGV_CHILD)),
                ((self.UAV_PARENT, "wrong_uav_child"), (self.UGV_PARENT, self.UGV_CHILD)),
            ]
        ):
            stamp = start + rospy.Duration(0.05 * index)
            self.uav_pub.publish(self.make_odom(stamp, *frames[0], z=2.0))
            self.ugv_pub.publish(self.make_odom(stamp, *frames[1]))
        rospy.sleep(0.1)
        self.assertEqual(
            rospy.wait_for_message(
                "/air_ground/registration/status", String, timeout=2.0
            ).data,
            "CAPTURING_ORIGIN",
        )

        origin_start = start + rospy.Duration(0.3)
        for index in range(3):
            self.publish_odometry_pair(origin_start + rospy.Duration(0.05 * index))
        acquiring = rospy.wait_for_message(
            "/air_ground/registration/status", String, timeout=5.0
        )
        self.assertEqual(acquiring.data, "ACQUIRING_REGISTRATION")

        for index, uav_frames in enumerate(
            [("wrong_uav_parent", self.UAV_CHILD), (self.UAV_PARENT, "wrong_uav_child")]
        ):
            stamp = start + rospy.Duration(0.8 + 0.3 * index)
            self.uav_pub.publish(self.make_odom(stamp, *uav_frames, z=2.0))
            self.ugv_pub.publish(
                self.make_odom(stamp, self.UGV_PARENT, self.UGV_CHILD)
            )
            self.observation_pub.publish(
                self.make_observation(stamp, translation_matrix((0.0, 0.0, 1.0)))
            )
        rospy.sleep(0.2)
        self.assertEqual(
            rospy.wait_for_message(
                "/air_ground/registration/status", String, timeout=2.0
            ).data,
            "ACQUIRING_REGISTRATION",
        )
        self.assertEqual(
            rospy.wait_for_message(
                "/air_ground/registration/revision", UInt32, timeout=2.0
            ).data,
            0,
        )

        expected_x, expected_y, expected_yaw = 2.0, -1.0, 0.35
        for index, offset in enumerate([-0.006, 0.004, -0.002, 0.005]):
            stamp = start + rospy.Duration(1.6 + 0.3 * index)
            self.publish_interpolated_sample(
                stamp,
                (
                    expected_x + offset,
                    expected_y - offset,
                    expected_yaw + offset * 0.2,
                ),
            )

        status = rospy.wait_for_message(
            "/air_ground/registration/status", String, timeout=5.0
        )
        self.assertEqual(status.data, "FROZEN")
        self.assertTrue(
            rospy.wait_for_message(
                "/air_ground/registration/frozen", Bool, timeout=2.0
            ).data
        )
        self.assertTrue(
            rospy.wait_for_message(
                "/air_ground/registration/valid", Bool, timeout=2.0
            ).data
        )
        self.assertEqual(
            rospy.wait_for_message(
                "/air_ground/registration/inlier_count", UInt32, timeout=2.0
            ).data,
            4,
        )
        self.assertEqual(
            rospy.wait_for_message(
                "/air_ground/registration/revision", UInt32, timeout=2.0
            ).data,
            1,
        )

        estimate = rospy.wait_for_message(
            "/air_ground/registration/estimate",
            PoseWithCovarianceStamped,
            timeout=2.0,
        )
        self.assertEqual(estimate.header.frame_id, "test_origin")
        self.assertLess(abs(estimate.pose.pose.position.x - expected_x), 0.03)
        self.assertLess(abs(estimate.pose.pose.position.y - expected_y), 0.03)
        quaternion = estimate.pose.pose.orientation
        yaw = euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )[2]
        self.assertLess(abs(math.atan2(math.sin(yaw - expected_yaw), math.cos(yaw - expected_yaw))), 0.01)
        covariance_indices = (0, 1, 5)
        planar_covariance = np.array(
            [
                [
                    estimate.pose.covariance[target_row * 6 + target_column]
                    for target_column in covariance_indices
                ]
                for target_row in covariance_indices
            ]
        )
        np.testing.assert_allclose(
            planar_covariance,
            np.array(
                [
                    [1.089813997879532e-04, -6.337021007873122e-06, 1.152242238294521e-06],
                    [-6.337021007873122e-06, 1.077442631452827e-04, -1.340244215354868e-06],
                    [1.152242238294521e-06, -1.340244215354868e-06, 2.551244577052707e-05],
                ]
            ),
            rtol=0.0,
            atol=1e-10,
        )

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        ugv_transform = tf_buffer.lookup_transform(
            "test_origin", "test_ugv_odom", rospy.Time(0), rospy.Duration(3.0)
        )
        self.assertLess(abs(ugv_transform.transform.translation.x - expected_x), 0.03)
        self.assertLess(abs(ugv_transform.transform.translation.y - expected_y), 0.03)
        uav_transform = tf_buffer.lookup_transform(
            "test_origin", "test_uav_odom", rospy.Time(0), rospy.Duration(3.0)
        )
        self.assertAlmostEqual(uav_transform.transform.translation.z, -2.0, places=6)

        for index in range(6):
            stamp = rospy.Time.now() + rospy.Duration(0.05 * index)
            self.publish_odometry_pair(stamp)
            contradictory = self.pose_matrix(
                (-4.0, 6.0, 1.0), quaternion_from_euler(0.0, 0.0, -1.0)
            )
            self.observation_pub.publish(
                self.make_observation(stamp, contradictory)
            )
        rospy.sleep(0.3)
        self.assertEqual(
            rospy.wait_for_message(
                "/air_ground/registration/revision", UInt32, timeout=2.0
            ).data,
            1,
        )
        frozen_estimate = rospy.wait_for_message(
            "/air_ground/registration/estimate",
            PoseWithCovarianceStamped,
            timeout=2.0,
        )
        self.assertLess(abs(frozen_estimate.pose.pose.position.x - expected_x), 0.03)
        self.assertLess(abs(frozen_estimate.pose.pose.position.y - expected_y), 0.03)

    def test_repeated_windows_update_reject_and_degrade_without_losing_registration(self):
        prefix = "/test/repeated/"
        start = rospy.Time.now()
        for index in range(3):
            stamp = start + rospy.Duration(0.05 * index)
            self.repeated_uav_pub.publish(
                self.make_odom(stamp, self.UAV_PARENT, self.UAV_CHILD, z=2.0)
            )
            self.repeated_ugv_pub.publish(
                self.make_odom(stamp, self.UGV_PARENT, self.UGV_CHILD)
            )
            rospy.sleep(0.03)
        self.wait_for_status(prefix + "status", "ACQUIRING_INITIAL")

        first_target = (2.0, -1.0, 0.35)
        self.publish_repeated_window(start + rospy.Duration(1.0), first_target)
        self.wait_for_revision(prefix + "revision", 1)
        self.wait_for_status(prefix + "status", "TRACKING")

        second_target = (2.01, -1.005, 0.352)
        self.publish_repeated_window(start + rospy.Duration(3.0), second_target)
        self.wait_for_revision(prefix + "revision", 2)
        self.wait_for_status(prefix + "status", "TRACKING")
        accepted_estimate = rospy.wait_for_message(
            prefix + "estimate", PoseWithCovarianceStamped, timeout=2.0
        )
        accepted_trace = sum(
            accepted_estimate.pose.covariance[index] for index in (0, 7, 35)
        )
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        accepted_tf = tf_buffer.lookup_transform(
            "test_repeated_origin",
            "test_repeated_ugv_odom",
            rospy.Time(0),
            rospy.Duration(3.0),
        )

        self.publish_repeated_window(
            start + rospy.Duration(5.0), (20.0, -30.0, 2.5)
        )
        self.wait_for_status(prefix + "status", "REJECTED")
        self.assertEqual(
            rospy.wait_for_message(prefix + "revision", UInt32, timeout=2.0).data,
            2,
        )
        rejected_estimate = rospy.wait_for_message(
            prefix + "estimate", PoseWithCovarianceStamped, timeout=2.0
        )
        self.assertAlmostEqual(
            rejected_estimate.pose.pose.position.x,
            accepted_estimate.pose.pose.position.x,
            places=9,
        )
        self.assertAlmostEqual(
            rejected_estimate.pose.pose.position.y,
            accepted_estimate.pose.pose.position.y,
            places=9,
        )
        rejected_tf = tf_buffer.lookup_transform(
            "test_repeated_origin",
            "test_repeated_ugv_odom",
            rospy.Time(0),
            rospy.Duration(3.0),
        )
        self.assertAlmostEqual(
            rejected_tf.transform.translation.x,
            accepted_tf.transform.translation.x,
            places=9,
        )
        self.assertAlmostEqual(
            rejected_tf.transform.translation.y,
            accepted_tf.transform.translation.y,
            places=9,
        )
        self.assertTrue(
            math.isfinite(
                rospy.wait_for_message(
                    prefix + "innovation", Float64, timeout=2.0
                ).data
            )
        )

        hidden_stamp = start + rospy.Duration(8.0)
        self.repeated_uav_pub.publish(
            self.make_odom(hidden_stamp, self.UAV_PARENT, self.UAV_CHILD, z=2.0)
        )
        self.repeated_ugv_pub.publish(
            self.make_odom(hidden_stamp, self.UGV_PARENT, self.UGV_CHILD)
        )
        self.wait_for_status(prefix + "status", "DEGRADED")
        self.assertEqual(
            rospy.wait_for_message(prefix + "revision", UInt32, timeout=2.0).data,
            2,
        )
        degraded_estimate = rospy.wait_for_message(
            prefix + "estimate", PoseWithCovarianceStamped, timeout=2.0
        )
        degraded_trace = sum(
            degraded_estimate.pose.covariance[index] for index in (0, 7, 35)
        )
        self.assertGreater(degraded_trace, accepted_trace)


if __name__ == "__main__":
    rostest.rosrun(
        "air_ground_coordinate_transform", "registration_node_test", RegistrationNodeTest
    )
