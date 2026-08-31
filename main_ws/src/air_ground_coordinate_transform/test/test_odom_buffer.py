#!/usr/bin/env python3

import math
import subprocess
import sys
import unittest
from types import SimpleNamespace

import numpy as np

from air_ground_coordinate_transform.odom_buffer import OdomBuffer


class Stamp:
    def __init__(self, seconds):
        self._seconds = seconds

    def to_sec(self):
        return self._seconds


class OdomBufferTest(unittest.TestCase):
    def test_returns_pose_at_exact_timestamp(self):
        buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
        buffer.append(1.0, 2.0, 3.0, 4.0, 0.25)

        np.testing.assert_allclose(buffer.interpolate(1.0), [2.0, 3.0, 0.25])

    def test_interpolates_translation_linearly(self):
        buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
        buffer.append(1.00, 0.0, -2.0, 3.0, 0.0)
        buffer.append(1.10, 2.0, 4.0, 5.0, 0.0)

        np.testing.assert_allclose(buffer.interpolate(1.05), [1.0, 1.0, 0.0])

    def test_interpolates_yaw_across_pi_by_shortest_path(self):
        buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
        buffer.append(1.00, 0.0, 0.0, 0.0, math.radians(179.0))
        buffer.append(1.10, 1.0, 0.0, 0.0, math.radians(-179.0))

        result = buffer.interpolate(Stamp(1.05))

        self.assertAlmostEqual(abs(result[2]), math.pi, places=5)

    def test_returns_preceding_sample_within_max_bracket(self):
        buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
        buffer.append(1.0, 2.0, 3.0, 0.0, 0.25)

        np.testing.assert_allclose(buffer.interpolate(1.05), [2.0, 3.0, 0.25])

    def test_rejects_preceding_sample_outside_max_bracket(self):
        buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
        buffer.append(1.0, 2.0, 3.0, 0.0, 0.25)

        self.assertIsNone(buffer.interpolate(1.09))

    def test_rejects_interpolation_when_one_side_is_outside_max_bracket(self):
        buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
        buffer.append(1.0, 0.0, 0.0, 0.0, 0.0)
        buffer.append(1.1, 1.0, 0.0, 0.0, 0.0)

        self.assertIsNone(buffer.interpolate(1.09))

    def test_rejects_interpolation_when_following_sample_is_too_far(self):
        buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
        buffer.append(1.0, 0.0, 0.0, 0.0, 0.0)
        buffer.append(1.2, 2.0, 0.0, 0.0, 0.0)

        self.assertIsNone(buffer.interpolate(1.05))

    def test_distance_since_accumulates_planar_distance(self):
        buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
        buffer.append(1.0, 0.0, 0.0, 0.0, 0.0)
        buffer.append(1.1, 3.0, 4.0, 10.0, 0.0)
        buffer.append(1.2, 6.0, 8.0, 20.0, 0.0)

        self.assertAlmostEqual(buffer.distance_since(1.0), 10.0)

    def test_distance_since_interpolates_non_exact_start(self):
        buffer = OdomBuffer(maxlen=10, max_bracket=0.6)
        buffer.append(1.0, 0.0, 0.0, 0.0, 0.0)
        buffer.append(2.0, 2.0, 0.0, 0.0, 0.0)
        buffer.append(3.0, 2.0, 3.0, 0.0, 0.0)

        self.assertAlmostEqual(buffer.distance_since(1.5), 4.0)

    def test_maxlen_evicts_oldest_sample(self):
        buffer = OdomBuffer(maxlen=2, max_bracket=0.08)
        buffer.append(1.0, 1.0, 0.0, 0.0, 0.0)
        buffer.append(2.0, 2.0, 0.0, 0.0, 0.0)
        buffer.append(3.0, 3.0, 0.0, 0.0, 0.0)

        self.assertIsNone(buffer.interpolate(1.0))
        np.testing.assert_allclose(buffer.interpolate(2.0), [2.0, 0.0, 0.0])

    def test_append_odometry_converts_message_without_ros_imports(self):
        half_yaw = 0.25
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=Stamp(2.0)),
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1.0, y=-2.0, z=3.0),
                    orientation=SimpleNamespace(
                        x=0.0,
                        y=0.0,
                        z=math.sin(half_yaw),
                        w=math.cos(half_yaw),
                    ),
                )
            ),
        )
        buffer = OdomBuffer(maxlen=10, max_bracket=0.08)

        buffer.append_odometry(message)

        np.testing.assert_allclose(buffer.interpolate(2.0), [1.0, -2.0, 0.5])

    def test_interpolate_full_preserves_translation_and_slerps_quaternion(self):
        def message(stamp, position, quaternion):
            return SimpleNamespace(
                header=SimpleNamespace(stamp=Stamp(stamp)),
                pose=SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(
                            x=position[0], y=position[1], z=position[2]
                        ),
                        orientation=SimpleNamespace(
                            x=quaternion[0],
                            y=quaternion[1],
                            z=quaternion[2],
                            w=quaternion[3],
                        ),
                    )
                ),
            )

        buffer = OdomBuffer(maxlen=10, max_bracket=0.6)
        buffer.append_odometry(message(1.0, [0.0, -2.0, 1.0], [0.0, 0.0, 0.0, 1.0]))
        buffer.append_odometry(
            message(
                2.0,
                [2.0, 4.0, 3.0],
                [math.sin(math.pi / 4.0), 0.0, 0.0, math.cos(math.pi / 4.0)],
            )
        )

        interpolate_full = getattr(buffer, "interpolate_full", None)
        self.assertIsNotNone(interpolate_full)
        result = interpolate_full(1.5)

        np.testing.assert_allclose(result[:3], [1.0, 1.0, 2.0], atol=1e-12)
        np.testing.assert_allclose(
            result[3:],
            [math.sin(math.pi / 8.0), 0.0, 0.0, math.cos(math.pi / 8.0)],
            atol=1e-12,
        )
        np.testing.assert_allclose(buffer.interpolate(1.5), [1.0, 1.0, 0.0])

    def test_module_import_does_not_load_ros_modules(self):
        script = """
import builtins

real_import = builtins.__import__
forbidden = {"geometry_msgs", "nav_msgs", "roslib", "rospy", "tf", "tf2_ros"}

def reject_ros_import(name, *args, **kwargs):
    if name.split(".", 1)[0] in forbidden:
        raise RuntimeError("ROS import attempted: " + name)
    return real_import(name, *args, **kwargs)

builtins.__import__ = reject_ros_import
import air_ground_coordinate_transform.odom_buffer
"""

        result = subprocess.run(
            [sys.executable, "-c", script],
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
