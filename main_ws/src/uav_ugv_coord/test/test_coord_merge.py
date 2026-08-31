#!/usr/bin/env python3
import os
import sys
import unittest

import numpy as np
import rospy
import tf.transformations as tr
from nav_msgs.msg import Odometry


sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from coord_merge_node import (  # noqa: E402
    OdomBuffer,
    TransformAverage,
    merge_coordinate_frames,
    xyz_rpy_to_matrix,
)


def odom(stamp, xyz, yaw):
    msg = Odometry()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.pose.pose.position.x = xyz[0]
    msg.pose.pose.position.y = xyz[1]
    msg.pose.pose.position.z = xyz[2]
    quat = tr.quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.pose.orientation.x = quat[0]
    msg.pose.pose.orientation.y = quat[1]
    msg.pose.pose.orientation.z = quat[2]
    msg.pose.pose.orientation.w = quat[3]
    return msg


class CoordMergeTest(unittest.TestCase):

    def test_odom_interpolates_translation_and_rotation(self):
        buffer = OdomBuffer('test', 1.0)
        buffer.add(odom(1.0, [0.0, 0.0, 0.0], 0.0))
        buffer.add(odom(3.0, [2.0, 4.0, 6.0], np.pi))

        pose, age = buffer.lookup(rospy.Time.from_sec(2.0))

        np.testing.assert_allclose(pose[:3, 3], [1.0, 2.0, 3.0])
        self.assertAlmostEqual(abs(tr.euler_from_matrix(pose)[2]),
                               np.pi / 2.0)
        self.assertAlmostEqual(age.to_sec(), 1.0)

    def test_coordinate_chain_is_invariant_to_uav_motion(self):
        T_bU_cam = xyz_rpy_to_matrix(
            [0.0, 0.0, -0.17], [np.pi, 0.0, -np.pi / 2.0])
        T_bG_tag = xyz_rpy_to_matrix(
            [0.0, 0.0, 0.30], [np.pi, 0.0, np.pi / 2.0])
        T_mG_bG = xyz_rpy_to_matrix(
            [0.4, -0.2, 0.0], [0.0, 0.0, 0.15])
        T_mU_mG = xyz_rpy_to_matrix(
            [1.5, 0.3, -0.1], [0.0, 0.0, -0.25])
        expected_ugv = T_mU_mG.dot(T_mG_bG)

        for xyz, rpy in (
                ([0.0, 0.0, 3.0], [0.0, 0.0, 0.0]),
                ([2.0, -1.0, 6.0], [0.1, -0.08, 0.4]),
                ([-1.0, 2.0, 4.0], [-0.05, 0.12, -0.7])):
            T_mU_bU = xyz_rpy_to_matrix(xyz, rpy)
            T_cam_tag = (np.linalg.inv(T_mU_bU.dot(T_bU_cam))
                         .dot(expected_ugv).dot(T_bG_tag))

            actual_ugv, actual_map = merge_coordinate_frames(
                T_mU_bU, T_mG_bG, T_bU_cam, T_cam_tag, T_bG_tag)

            np.testing.assert_allclose(actual_ugv, expected_ugv, atol=1e-9)
            np.testing.assert_allclose(actual_map, T_mU_mG, atol=1e-9)

    def test_transform_average_handles_quaternion_sign(self):
        average = TransformAverage()
        first = xyz_rpy_to_matrix([1.0, 2.0, 3.0], [0.0, 0.0, 0.2])
        second = xyz_rpy_to_matrix([3.0, 4.0, 5.0], [0.0, 0.0, 0.2])

        average.add(first)
        result = average.add(second)

        np.testing.assert_allclose(result[:3, 3], [2.0, 3.0, 4.0])
        self.assertAlmostEqual(tr.euler_from_matrix(result)[2], 0.2)


if __name__ == '__main__':
    unittest.main()
