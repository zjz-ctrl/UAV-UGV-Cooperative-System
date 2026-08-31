"""Bounded odometry storage and interpolation without ROS import side effects."""

import bisect
import math
from collections import deque

import numpy as np

from .se2 import wrap_angle


def _stamp_seconds(stamp):
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    return float(stamp)


def _normalize_quaternion(quaternion):
    quaternion = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return quaternion / norm


def _slerp(first, second, fraction):
    first = _normalize_quaternion(first)
    second = _normalize_quaternion(second)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    if dot > 0.9995:
        return _normalize_quaternion(first + fraction * (second - first))
    angle = math.acos(max(-1.0, min(1.0, dot)))
    scale = math.sin(angle)
    return (
        math.sin((1.0 - fraction) * angle) * first
        + math.sin(fraction * angle) * second
    ) / scale


class OdomBuffer:
    def __init__(self, maxlen, max_bracket):
        self._samples = deque(maxlen=int(maxlen))
        self._max_bracket = float(max_bracket)

    def append(self, stamp, x, y, z, yaw):
        half_yaw = 0.5 * float(yaw)
        self._samples.append(
            (
                _stamp_seconds(stamp),
                float(x),
                float(y),
                float(z),
                0.0,
                0.0,
                math.sin(half_yaw),
                math.cos(half_yaw),
            )
        )

    def append_odometry(self, message):
        pose = message.pose.pose
        quaternion = pose.orientation
        normalized = _normalize_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        self._samples.append(
            (
                _stamp_seconds(message.header.stamp),
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                *normalized,
            )
        )

    def interpolate(self, stamp):
        pose = self.interpolate_full(stamp)
        if pose is None:
            return None
        x, y = pose[:2]
        quaternion = pose[3:]
        yaw = math.atan2(
            2.0
            * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]),
            1.0
            - 2.0
            * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]),
        )
        return np.array([x, y, wrap_angle(yaw)])

    def interpolate_full(self, stamp):
        target = _stamp_seconds(stamp)
        samples = list(self._samples)
        if not samples:
            return None

        index = bisect.bisect_left([sample[0] for sample in samples], target)
        if index < len(samples) and samples[index][0] == target:
            sample = samples[index]
            return np.asarray(sample[1:], dtype=float)

        if index == 0:
            return None

        previous = samples[index - 1]
        if index == len(samples):
            if target - previous[0] > self._max_bracket:
                return None
            return np.asarray(previous[1:], dtype=float)

        following = samples[index]
        if (
            target - previous[0] > self._max_bracket
            or following[0] - target > self._max_bracket
        ):
            return None

        fraction = (target - previous[0]) / (following[0] - previous[0])
        translation = np.asarray(previous[1:4]) + fraction * (
            np.asarray(following[1:4]) - np.asarray(previous[1:4])
        )
        quaternion = _slerp(previous[4:8], following[4:8], fraction)
        return np.concatenate((translation, quaternion))

    def distance_since(self, stamp):
        target = _stamp_seconds(stamp)
        start = self.interpolate(target)
        if start is None:
            return None

        distance = 0.0
        previous_x, previous_y = start[:2]
        for sample in self._samples:
            if sample[0] <= target:
                continue
            distance += math.hypot(sample[1] - previous_x, sample[2] - previous_y)
            previous_x, previous_y = sample[1], sample[2]
        return distance
