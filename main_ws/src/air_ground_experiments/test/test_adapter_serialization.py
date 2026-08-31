#!/usr/bin/env python3

import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np


PACKAGE_SOURCE = Path(__file__).parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from air_ground_experiments.frame_perturbation import (
    FramePerturbation,
    diagnostic_json,
    odometry_record,
    populate_odometry,
    truth_json,
)


def vector(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def odometry_fixture():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 7.5), frame_id="map"),
        child_frame_id="base_link",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=vector(1.0, 2.0, 3.0),
                orientation=SimpleNamespace(x=0.1, y=0.2, z=0.3, w=0.9),
            ),
            covariance=list(range(36)),
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=vector(4.0, 5.0, 6.0),
                angular=vector(7.0, 8.0, 9.0),
            ),
            covariance=list(range(100, 136)),
        ),
    )


class AdapterSerializationTest(unittest.TestCase):
    def test_odometry_record_reads_complete_pose_twist_and_covariances(self):
        stamp, pose, twist = odometry_record(odometry_fixture())

        self.assertEqual(stamp, 7.5)
        np.testing.assert_array_equal(pose["position"], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(pose["orientation"], [0.1, 0.2, 0.3, 0.9])
        np.testing.assert_array_equal(pose["covariance"].reshape(-1), range(36))
        np.testing.assert_array_equal(twist["linear"], [4.0, 5.0, 6.0])
        np.testing.assert_array_equal(twist["angular"], [7.0, 8.0, 9.0])
        np.testing.assert_array_equal(twist["covariance"].reshape(-1), range(100, 136))

    def test_populate_odometry_preserves_stamp_child_and_writes_all_fields(self):
        message = odometry_fixture()
        original_stamp = message.header.stamp
        pose = {
            "position": np.array([10.0, 11.0, 12.0]),
            "orientation": np.array([0.4, 0.3, 0.2, 0.1]),
            "covariance": np.arange(36, dtype=float).reshape(6, 6) + 2.0,
        }
        twist = {
            "linear": np.array([13.0, 14.0, 15.0]),
            "angular": np.array([16.0, 17.0, 18.0]),
            "covariance": np.arange(36, dtype=float).reshape(6, 6) + 50.0,
        }

        populate_odometry(message, pose, twist, "independent_uav_frame")

        self.assertIs(message.header.stamp, original_stamp)
        self.assertEqual(message.header.frame_id, "independent_uav_frame")
        self.assertEqual(message.child_frame_id, "base_link")
        self.assertEqual(message.pose.pose.position.z, 12.0)
        self.assertEqual(message.pose.pose.orientation.x, 0.4)
        self.assertEqual(message.twist.twist.linear.z, 15.0)
        self.assertEqual(message.twist.twist.angular.z, 18.0)
        self.assertEqual(message.pose.covariance, list(range(2, 38)))
        self.assertEqual(message.twist.covariance, list(range(50, 86)))

    def test_truth_json_exposes_seed_stamp_frames_and_transform(self):
        result = json.loads(
            truth_json([1.0, -2.0, 0.3], 99, 4.5, "map", "independent")
        )

        self.assertEqual(result["seed"], 99)
        self.assertEqual(result["stamp"], 4.5)
        self.assertEqual(result["source_frame"], "map")
        self.assertEqual(result["destination_frame"], "independent")
        self.assertEqual(result["transform_xyyaw"], [1.0, -2.0, 0.3])

    def test_observation_diagnostic_explicitly_reports_original_stamp_and_delay(self):
        result = json.loads(
            diagnostic_json(
                image_stamp=12.0,
                release_time=12.7,
                injected_delay=0.7,
                outlier_xyyaw=(1.0, 2.0, 0.2),
                seed=55,
            )
        )

        self.assertEqual(result["image_stamp"], 12.0)
        self.assertEqual(result["injected_delay_seconds"], 0.7)
        self.assertEqual(result["scheduled_release"], 12.7)
        self.assertEqual(result["seed"], 55)


# Hand-computed fixture: yaw = +90 deg, R = [[0,-1,0],[1,0,0],[0,0,1]],
# J = blockdiag(R, R). Input covariance chosen with asymmetric cross terms;
# expected matrix below was expanded by hand as J C J^T and is NOT computed
# by any production helper.
CROSS_COVARIANCE_INPUT = [
    [2.0,  1.0, 0.0, 0.0,  0.0, 0.0],
    [1.0,  3.0, 0.0, 0.0,  0.0, 0.0],
    [0.0,  0.0, 2.0, 0.0,  0.5, 0.0],
    [0.0,  0.0, 0.0, 5.0, -2.0, 0.0],
    [0.0,  0.0, 0.5, -2.0, 4.0, 0.0],
    [0.0,  0.0, 0.0, 0.0,  0.0, 3.0],
]
CROSS_COVARIANCE_EXPECTED_PARENT = [
    [3.0, -1.0,  0.0,  0.0,  0.0, 0.0],
    [-1.0, 2.0,  0.0,  0.0,  0.0, 0.0],
    [0.0,  0.0,  2.0, -0.5,  0.0, 0.0],
    [0.0,  0.0, -0.5,  4.0,  2.0, 0.0],
    [0.0,  0.0,  0.0,  2.0,  5.0, 0.0],
    [0.0,  0.0,  0.0,  0.0,  0.0, 3.0],
]


class TwistConventionTest(unittest.TestCase):
    def make_records(self):
        pose = {
            "position": np.array([1.0, 2.0, 3.0]),
            "orientation": np.array([0.0, 0.0, 0.0, 1.0]),
            "covariance": np.array(CROSS_COVARIANCE_INPUT),
        }
        twist = {
            "linear": np.array([1.0, 2.0, 3.0]),
            "angular": np.array([4.0, 5.0, 6.0]),
            "covariance": np.array(CROSS_COVARIANCE_INPUT),
        }
        return pose, twist

    def test_parent_convention_rotates_twist_and_cross_terms(self):
        perturbation = FramePerturbation(
            [0.0, 0.0, math.pi / 2.0], [0.0] * 3, 7,
        )
        pose, twist = self.make_records()

        _, transformed_twist, _ = perturbation.transform_odom(
            pose, twist, 0.0, twist_convention="parent"
        )

        np.testing.assert_allclose(transformed_twist["linear"], [-2.0, 1.0, 3.0])
        np.testing.assert_allclose(transformed_twist["angular"], [-5.0, 4.0, 6.0])
        np.testing.assert_allclose(
            transformed_twist["covariance"], CROSS_COVARIANCE_EXPECTED_PARENT,
            atol=1e-12,
        )

    def test_body_convention_preserves_twist_entirely(self):
        perturbation = FramePerturbation(
            [0.0, 0.0, math.pi / 2.0], [0.0] * 3, 7,
        )
        pose, twist = self.make_records()

        _, transformed_twist, _ = perturbation.transform_odom(
            pose, twist, 0.0, twist_convention="body"
        )

        np.testing.assert_array_equal(transformed_twist["linear"], [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(transformed_twist["angular"], [4.0, 5.0, 6.0])
        np.testing.assert_array_equal(
            transformed_twist["covariance"], CROSS_COVARIANCE_INPUT
        )

    def test_populate_labels_child_frame_honestly_per_convention(self):
        message_body = odometry_fixture()
        message_parent = odometry_fixture()
        pose = {
            "position": np.zeros(3),
            "orientation": np.array([0.0, 0.0, 0.0, 1.0]),
            "covariance": np.eye(6),
        }

        populate_odometry(
            message_body, pose, {
                "linear": np.ones(3), "angular": np.ones(3),
                "covariance": np.eye(6),
            },
            "experiment_frame", twist_convention="body",
        )
        populate_odometry(
            message_parent, pose, {
                "linear": np.ones(3), "angular": np.ones(3),
                "covariance": np.eye(6),
            },
            "experiment_frame", twist_convention="parent",
        )

        self.assertEqual(message_body.child_frame_id, "base_link")
        self.assertEqual(message_parent.child_frame_id, "experiment_frame")


if __name__ == "__main__":
    unittest.main()
