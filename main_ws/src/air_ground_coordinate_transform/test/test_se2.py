#!/usr/bin/env python3

import math
import unittest

import numpy as np

from air_ground_coordinate_transform.se2 import (
    compose,
    inverse,
    matrix_from_xyyaw,
    transform_pose_covariance,
    wrap_angle,
    wrap_xyyaw,
    xyyaw_from_matrix,
)


class Se2Test(unittest.TestCase):
    def test_wrap_angle_normalizes_to_principal_range(self):
        self.assertAlmostEqual(wrap_angle(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_angle(-3.0 * math.pi), -math.pi)

    def test_wrap_xyyaw_normalizes_heading_only(self):
        value = np.array([4.0, -2.0, 3.0 * math.pi])

        wrapped = wrap_xyyaw(value)

        np.testing.assert_allclose(wrapped, np.array([4.0, -2.0, -math.pi]))
        np.testing.assert_allclose(value, np.array([4.0, -2.0, 3.0 * math.pi]))

    def test_matrix_round_trip_preserves_pose(self):
        value = np.array([1.2, -0.4, 0.7])

        result = xyyaw_from_matrix(matrix_from_xyyaw(*value))

        np.testing.assert_allclose(result, value, atol=1e-9)

    def test_compose_with_identity_preserves_transform(self):
        matrix = matrix_from_xyyaw(1.2, -0.4, 0.7)

        np.testing.assert_allclose(compose(matrix, np.eye(3)), matrix, atol=1e-9)

    def test_compose_with_inverse_returns_identity(self):
        value = np.array([1.2, -0.4, 0.7])
        matrix = matrix_from_xyyaw(*value)

        np.testing.assert_allclose(
            compose(matrix, inverse(matrix)), np.eye(3), atol=1e-9
        )

    def test_compose_applies_second_transform_then_first_transform(self):
        first = matrix_from_xyyaw(1.0, 2.0, math.pi / 2.0)
        second = matrix_from_xyyaw(3.0, 4.0, -math.pi / 2.0)
        expected = np.array(
            [
                [1.0, 0.0, -3.0],
                [0.0, 1.0, 5.0],
                [0.0, 0.0, 1.0],
            ]
        )

        np.testing.assert_allclose(compose(first, second), expected, atol=1e-9)

    def test_transform_covariance_includes_heading_lever_arm(self):
        point = np.array([10.0, 0.0, 0.0])
        point_cov = np.zeros((3, 3))
        tf_mean = np.zeros(3)
        tf_cov = np.diag([0.01, 0.01, math.radians(1.0) ** 2])

        mean, covariance = transform_pose_covariance(
            point, point_cov, tf_mean, tf_cov
        )

        np.testing.assert_allclose(mean, point)
        self.assertGreater(covariance[1, 1], 0.03)

    def test_transform_covariance_propagates_full_correlated_covariances(self):
        point = np.array([2.0, -1.0, 0.3])
        point_covariance = np.array(
            [
                [0.4, 0.05, 0.02],
                [0.05, 0.3, -0.01],
                [0.02, -0.01, 0.04],
            ]
        )
        transform_mean = np.array([3.0, 4.0, math.pi / 2.0])
        transform_covariance = np.array(
            [
                [0.2, 0.03, -0.02],
                [0.03, 0.1, 0.01],
                [-0.02, 0.01, 0.05],
            ]
        )
        expected_mean = np.array([4.0, 6.0, 1.8707963267948966])
        expected_covariance = np.array(
            [
                [0.90, -0.27, -0.19],
                [-0.27, 0.65, 0.12],
                [-0.19, 0.12, 0.09],
            ]
        )

        mean, covariance = transform_pose_covariance(
            point,
            point_covariance,
            transform_mean,
            transform_covariance,
        )

        np.testing.assert_allclose(mean, expected_mean, atol=1e-9)
        np.testing.assert_allclose(covariance, expected_covariance, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
