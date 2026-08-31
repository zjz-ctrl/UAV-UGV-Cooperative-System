#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock


def message_module(name, **messages):
    package = ModuleType(name)
    child = ModuleType(name + ".msg")
    for message_name, message_type in messages.items():
        setattr(child, message_name, message_type)
    package.msg = child
    return package, child


def load_monitor_module():
    rospy = ModuleType("rospy")
    rospy.loginfo = lambda *_args: None
    tf2_ros = ModuleType("tf2_ros")
    geometry, geometry_msg = message_module(
        "geometry_msgs", PoseWithCovarianceStamped=object
    )
    nav, nav_msg = message_module("nav_msgs", Odometry=object)
    std, std_msg = message_module(
        "std_msgs", Bool=object, String=object, UInt32=object
    )
    registration_messages = ModuleType("air_ground_coordinate_transform.msg")
    registration_messages.RegistrationUpdate = object
    replacements = {
        "rospy": rospy,
        "tf2_ros": tf2_ros,
        "geometry_msgs": geometry,
        "geometry_msgs.msg": geometry_msg,
        "nav_msgs": nav,
        "nav_msgs.msg": nav_msg,
        "std_msgs": std,
        "std_msgs.msg": std_msg,
        "air_ground_coordinate_transform.msg": registration_messages,
    }
    script = (
        Path(__file__).parents[2]
        / "air_ground_bringup"
        / "scripts"
        / "ugv_coordinate_monitor.py"
    )
    specification = importlib.util.spec_from_file_location(
        "ugv_coordinate_monitor_under_test", script
    )
    module = importlib.util.module_from_spec(specification)
    with mock.patch.dict(sys.modules, replacements):
        specification.loader.exec_module(module)
    return module


def accepted_update_message(revision, x, y, yaw, variance_x, variance_y, variance_yaw):
    covariance = [0.0] * 36
    covariance[0] = variance_x
    covariance[7] = variance_y
    covariance[35] = variance_yaw
    return SimpleNamespace(
        revision=revision,
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            ),
            covariance=covariance,
        )
    )


class UgvCoordinateMonitorTest(unittest.TestCase):
    @staticmethod
    def monitor(module):
        target = module.UgvCoordinateMonitor.__new__(module.UgvCoordinateMonitor)
        target.registration_lock = threading.Lock()
        target.registration_revision = 0
        target.previous_registration = None
        return target

    def test_atomic_update_log_separates_uncertainty_and_registration_delta(self):
        module = load_monitor_module()
        target = self.monitor(module)
        callback = getattr(target, "accepted_update_callback", None)

        self.assertIsNotNone(callback)
        with mock.patch.object(module.rospy, "loginfo") as loginfo:
            callback(accepted_update_message(1, 1.0, 2.0, 0.5, 0.04, 0.09, 0.01))
            callback(
                accepted_update_message(2, 1.5, 1.0, 0.7, 0.01, 0.04, 0.0025)
            )

        arguments = loginfo.call_args.args
        self.assertIn("registration_delta", arguments[0])
        self.assertEqual(arguments[1], 2)
        self.assertAlmostEqual(arguments[2], 0.1)
        self.assertAlmostEqual(arguments[3], 0.2)
        self.assertAlmostEqual(arguments[4], math.degrees(0.05))
        self.assertAlmostEqual(arguments[5], 0.5)
        self.assertAlmostEqual(arguments[6], -1.0)
        self.assertAlmostEqual(arguments[7], math.degrees(0.2))

    def test_legacy_timer_estimate_before_revision_two_cannot_pollute_update(self):
        module = load_monitor_module()
        target = self.monitor(module)
        callback = getattr(target, "accepted_update_callback", None)

        self.assertIsNotNone(callback)
        with mock.patch.object(module.rospy, "loginfo") as loginfo:
            callback(
                accepted_update_message(1, 1.0, 0.0, 0.1, 0.04, 0.09, 0.01)
            )
            legacy_estimate_callback = getattr(target, "estimate_callback", None)
            if legacy_estimate_callback is not None:
                legacy_estimate_callback(
                    SimpleNamespace(
                        header=SimpleNamespace(seq=2),
                        pose=accepted_update_message(
                            1, 99.0, 99.0, 1.0, 1.0, 1.0, 1.0
                        ).pose,
                    )
                )
            legacy_revision_callback = getattr(target, "revision_callback", None)
            if legacy_revision_callback is not None:
                legacy_revision_callback(SimpleNamespace(data=2))
            callback(
                accepted_update_message(2, 2.0, -1.0, 0.2, 0.01, 0.04, 0.0025)
            )

        self.assertEqual(loginfo.call_count, 2)
        self.assertEqual(target.registration_revision, 2)
        self.assertEqual(target.previous_registration[:2], (2.0, -1.0))

    def test_duplicate_and_out_of_order_updates_are_ignored(self):
        module = load_monitor_module()
        target = self.monitor(module)
        callback = getattr(target, "accepted_update_callback", None)

        self.assertIsNotNone(callback)
        with mock.patch.object(module.rospy, "loginfo") as loginfo:
            callback(
                accepted_update_message(2, 2.0, -1.0, 0.2, 0.04, 0.09, 0.01)
            )
            callback(
                accepted_update_message(2, 99.0, 99.0, 1.0, 1.0, 1.0, 1.0)
            )
            callback(
                accepted_update_message(1, 88.0, 88.0, 0.9, 1.0, 1.0, 1.0)
            )

        self.assertEqual(loginfo.call_count, 1)
        self.assertEqual(target.previous_registration[:2], (2.0, -1.0))

    def test_revision_gap_is_accepted_atomically(self):
        module = load_monitor_module()
        target = self.monitor(module)
        callback = getattr(target, "accepted_update_callback", None)

        self.assertIsNotNone(callback)
        with mock.patch.object(module.rospy, "loginfo") as loginfo:
            callback(
                accepted_update_message(1, 1.0, 2.0, 0.1, 0.04, 0.09, 0.01)
            )
            callback(
                accepted_update_message(4, 4.0, 5.0, 0.4, 0.01, 0.04, 0.0025)
            )

        self.assertEqual(loginfo.call_count, 2)
        self.assertEqual(target.registration_revision, 4)
        self.assertEqual(target.previous_registration[:2], (4.0, 5.0))


if __name__ == "__main__":
    unittest.main()
