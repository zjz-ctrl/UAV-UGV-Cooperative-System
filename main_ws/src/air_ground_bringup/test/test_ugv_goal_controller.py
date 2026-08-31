#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import types
import unittest

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts" / "ugv_goal_controller.py"


class Namespace:
    def __init__(self, **values):
        self.__dict__.update(values)


class PoseStamped:
    def __init__(self):
        self.header = Namespace(stamp=None, frame_id="")
        self.pose = Namespace(
            position=Namespace(x=0.0, y=0.0, z=0.0),
            orientation=Namespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


class Twist:
    def __init__(self):
        self.linear = Namespace(x=0.0, y=0.0, z=0.0)
        self.angular = Namespace(x=0.0, y=0.0, z=0.0)


class Odometry:
    def __init__(self, frame_id="ugv_0/odom", x=0.0, y=0.0):
        self.header = Namespace(frame_id=frame_id)
        self.pose = Namespace(
            pose=Namespace(
                position=Namespace(x=x, y=y, z=0.0),
                orientation=Namespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeBuffer:
    def __init__(self):
        self.lookups = []
        self.translation = Namespace(x=0.0, y=0.0, z=0.0)

    def lookup_transform(self, target, source, stamp, timeout):
        self.lookups.append((target, source))
        return Namespace(
            transform=Namespace(
                translation=self.translation,
                rotation=Namespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )


class ControllerFrameContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parameters = {}
        cls.publishers = {}
        cls.subscribers = []
        cls.buffer = None

        rospy = types.ModuleType("rospy")
        rospy.get_param = lambda name, default=None: cls.parameters.get(name, default)

        def publisher(topic, _message_type, **_kwargs):
            result = FakePublisher(topic)
            cls.publishers[topic] = result
            return result

        rospy.Publisher = publisher
        rospy.Subscriber = lambda topic, message_type, callback, **kwargs: cls.subscribers.append(
            (topic, message_type, callback, kwargs)
        )
        rospy.Timer = lambda *_args, **_kwargs: None
        rospy.Duration = lambda value: value

        class Time:
            now_value = 100.0

            def __init__(self, value=0.0):
                self.value = value

            def __sub__(self, other):
                return Namespace(to_sec=lambda: self.value - other.value)

            @classmethod
            def now(cls):
                return cls(cls.now_value)

        rospy.Time = Time
        rospy.logwarn = lambda *_args, **_kwargs: None
        rospy.logwarn_throttle = lambda *_args, **_kwargs: None
        rospy.loginfo = lambda *_args, **_kwargs: None
        rospy.init_node = lambda *_args, **_kwargs: None
        rospy.spin = lambda: None

        tf2_ros = types.ModuleType("tf2_ros")

        def buffer_factory():
            cls.buffer = FakeBuffer()
            return cls.buffer

        tf2_ros.Buffer = buffer_factory
        tf2_ros.TransformListener = lambda _buffer: None
        tf2_ros.LookupException = RuntimeError
        tf2_ros.ConnectivityException = RuntimeError
        tf2_ros.ExtrapolationException = RuntimeError

        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.PoseStamped = PoseStamped
        geometry_msgs_msg.Twist = Twist
        geometry_msgs.msg = geometry_msgs_msg

        nav_msgs = types.ModuleType("nav_msgs")
        nav_msgs_msg = types.ModuleType("nav_msgs.msg")
        nav_msgs_msg.Odometry = Odometry
        nav_msgs.msg = nav_msgs_msg

        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Bool = type("Bool", (), {})
        std_msgs_msg.String = type("String", (), {})
        std_msgs.msg = std_msgs_msg

        tf = types.ModuleType("tf")
        transformations = types.ModuleType("tf.transformations")
        transformations.euler_from_quaternion = lambda _quaternion: (0.0, 0.0, 0.0)
        transformations.quaternion_from_euler = lambda *_angles: (0.0, 0.0, 0.0, 1.0)
        transformations.quaternion_matrix = lambda _quaternion: np.eye(4)
        tf.transformations = transformations

        cls.original_modules = {
            name: sys.modules.get(name)
            for name in (
                "rospy",
                "tf2_ros",
                "geometry_msgs",
                "geometry_msgs.msg",
                "nav_msgs",
                "nav_msgs.msg",
                "std_msgs",
                "std_msgs.msg",
                "tf",
                "tf.transformations",
            )
        }
        sys.modules.update(
            {
                "rospy": rospy,
                "tf2_ros": tf2_ros,
                "geometry_msgs": geometry_msgs,
                "geometry_msgs.msg": geometry_msgs_msg,
                "nav_msgs": nav_msgs,
                "nav_msgs.msg": nav_msgs_msg,
                "std_msgs": std_msgs,
                "std_msgs.msg": std_msgs_msg,
                "tf": tf,
                "tf.transformations": transformations,
            }
        )
        spec = importlib.util.spec_from_file_location("ugv_goal_controller_under_test", SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    @classmethod
    def tearDownClass(cls):
        for name, module in cls.original_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def setUp(self):
        self.parameters.clear()
        self.publishers.clear()
        self.subscribers.clear()
        type(self).buffer = None
        self.module.rospy.Time.now_value = 100.0

    def test_research_configuration_uses_one_experimental_odom_contract(self):
        self.parameters.update(
            {
                "~odom_topic": "/air_ground_experiment/ugv/odom",
                "~odom_frame": "air_ground_experiment/ugv_odom",
            }
        )
        controller = self.module.Controller()

        self.assertIn(
            "/air_ground_experiment/ugv/odom",
            [subscription[0] for subscription in self.subscribers],
        )

        goal = PoseStamped()
        goal.header.frame_id = "air_ground_experiment/ugv_odom"
        goal.pose.position.x = 2.0
        goal.pose.position.y = -1.0
        controller.goal_callback(goal)

        self.assertEqual(self.buffer.lookups, [])
        published = self.publishers["/air_ground/ugv_goal_odom"].messages[-1]
        self.assertEqual(published.header.frame_id, "air_ground_experiment/ugv_odom")

    def test_legacy_defaults_remain_raw_ugv_odometry(self):
        controller = self.module.Controller()
        self.assertIn("/ugv_0/odom", [subscription[0] for subscription in self.subscribers])

        goal = PoseStamped()
        goal.header.frame_id = "air_ground_origin"
        controller.goal_callback(goal)

        self.assertEqual(self.buffer.lookups[-1], ("ugv_0/odom", "air_ground_origin"))
        published = self.publishers["/air_ground/ugv_goal_odom"].messages[-1]
        self.assertEqual(published.header.frame_id, "ugv_0/odom")

    def test_nonidentity_tf_is_applied_for_a_goal_from_another_frame(self):
        controller = self.module.Controller()
        self.buffer.translation = Namespace(x=2.0, y=-3.0, z=0.0)
        goal = PoseStamped()
        goal.header.frame_id = "air_ground_origin"
        goal.pose.position.x = 1.0
        goal.pose.position.y = 4.0

        controller.goal_callback(goal)

        published = self.publishers["/air_ground/ugv_goal_odom"].messages[-1]
        self.assertEqual((published.pose.position.x, published.pose.position.y), (3.0, 1.0))

    def test_mismatched_odom_frame_is_rejected(self):
        self.parameters.update(
            {
                "~odom_topic": "/air_ground_experiment/ugv/odom",
                "~odom_frame": "air_ground_experiment/ugv_odom",
            }
        )
        controller = self.module.Controller()

        controller.odom_callback(Odometry(frame_id="ugv_0/odom"))

        self.assertIsNone(controller.odom)

    def test_nonfinite_odom_is_rejected(self):
        controller = self.module.Controller()

        controller.odom_callback(Odometry(x=float("nan")))

        self.assertIsNone(controller.odom)

    def test_stale_odom_stops_motion_and_reports_waiting(self):
        self.parameters["~maximum_odom_age"] = 0.25
        controller = self.module.Controller()
        controller.odom_callback(Odometry())
        goal = PoseStamped()
        goal.header.frame_id = "ugv_0/odom"
        goal.pose.position.x = 2.0
        controller.goal_callback(goal)
        controller.arrived = True
        self.publishers["/air_ground/ugv/arrived"].publish(True)
        self.module.rospy.Time.now_value = 101.0

        controller.tick(None)

        command = self.publishers["/ugv_0/cmd_vel"].messages[-1]
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(command.angular.z, 0.0)
        self.assertIs(controller.arrived, False)
        self.assertIs(
            self.publishers["/air_ground/ugv/arrived"].messages[-1],
            False,
        )
        self.assertEqual(
            self.publishers["/air_ground/ugv/status"].messages[-1],
            "WAITING_ODOMETRY",
        )


if __name__ == "__main__":
    unittest.main()
