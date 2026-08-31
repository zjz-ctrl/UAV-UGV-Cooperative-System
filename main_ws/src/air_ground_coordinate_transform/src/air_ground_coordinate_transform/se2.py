"""SE(2) transform and first-order covariance operations."""

import math

import numpy as np


def wrap_angle(angle):
    """Normalize an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def wrap_xyyaw(value):
    """Return an x/y/yaw vector with only its yaw normalized."""
    result = np.array(value, dtype=float, copy=True)
    result[2] = wrap_angle(result[2])
    return result


def matrix_from_xyyaw(x, y, yaw):
    """Construct a homogeneous planar transform."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array(
        [
            [c, -s, x],
            [s, c, y],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def xyyaw_from_matrix(matrix):
    """Extract x, y, and normalized yaw from a planar transform."""
    matrix = np.asarray(matrix, dtype=float)
    return np.array(
        [matrix[0, 2], matrix[1, 2], wrap_angle(math.atan2(matrix[1, 0], matrix[0, 0]))]
    )


def compose(first, second):
    """Compose two planar transforms and normalize the resulting yaw."""
    value = xyyaw_from_matrix(np.asarray(first) @ np.asarray(second))
    return matrix_from_xyyaw(*value)


def inverse(matrix):
    """Invert a planar transform."""
    matrix = np.asarray(matrix, dtype=float)
    rotation = matrix[:2, :2]
    translation = matrix[:2, 2]
    result = np.eye(3)
    result[:2, :2] = rotation.T
    result[:2, 2] = -(rotation.T @ translation)
    return matrix_from_xyyaw(*xyyaw_from_matrix(result))


def transform_pose_covariance(
    mean, covariance, transform_mean, transform_covariance
):
    """Apply an uncertain planar transform using first-order propagation."""
    mean = np.asarray(mean, dtype=float)
    covariance = np.asarray(covariance, dtype=float)
    transform_mean = np.asarray(transform_mean, dtype=float)
    transform_covariance = np.asarray(transform_covariance, dtype=float)

    px, py, _ = mean
    tx, ty, transform_yaw = transform_mean
    c = math.cos(transform_yaw)
    s = math.sin(transform_yaw)

    transformed_mean = wrap_xyyaw(
        np.array(
            [
                tx + c * px - s * py,
                ty + s * px + c * py,
                transform_yaw + mean[2],
            ]
        )
    )

    j_point = np.array(
        [
            [c, -s, -s * px - c * py],
            [s, c, c * px - s * py],
            [0.0, 0.0, 1.0],
        ]
    )
    j_transform = np.array(
        [
            [1.0, 0.0, -s * px - c * py],
            [0.0, 1.0, c * px - s * py],
            [0.0, 0.0, 1.0],
        ]
    )
    transformed_covariance = (
        j_point @ covariance @ j_point.T
        + j_transform @ transform_covariance @ j_transform.T
    )
    return transformed_mean, transformed_covariance
