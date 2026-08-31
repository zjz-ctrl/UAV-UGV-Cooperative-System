#!/usr/bin/env python3
"""Duck-typed ROS message layouts and a fake rospy for script-level tests."""

import importlib.util
import math
from pathlib import Path
import sys
import types


SCRIPTS = Path(__file__).parents[1] / "scripts"


class FakeTime:
    def __init__(self, seconds=0.0):
        self._seconds = float(seconds)

    def to_sec(self):
        return self._seconds

    def is_zero(self):
        return self._seconds == 0.0

    def __sub__(self, other):
        return FakeTime(self._seconds - other.to_sec())


class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class Point(Vector3):
    pass


class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = (float(v) for v in (x, y, z, w))


class Header:
    def __init__(self, stamp=None, frame_id=""):
        self.stamp = stamp if stamp is not None else FakeTime(0.0)
        self.frame_id = frame_id


class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()


class PoseWithCovariance:
    def __init__(self):
        self.pose = Pose()
        self.covariance = [0.0] * 36


class Twist:
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class TwistWithCovariance:
    def __init__(self):
        self.twist = Twist()
        self.covariance = [0.0] * 36


class Odometry:
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ""
        self.pose = PoseWithCovariance()
        self.twist = TwistWithCovariance()


class PoseWithCovarianceStamped:
    def __init__(self):
        self.header = Header()
        self.pose = PoseWithCovariance()


class String:
    def __init__(self, data=""):
        self.data = data


class PointStamped:
    def __init__(self):
        self.header = Header()
        self.point = Point()


class PositionCommand:
    TRAJECTORY_STATUS_EMPTY = 0
    TRAJECTORY_STATUS_READY = 1
    TRAJECTORY_STATUS_COMPLETED = 3
    TRAJECTROY_STATUS_ABORT = 4
    TRAJECTORY_STATUS_ILLEGAL_START = 5
    TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
    TRAJECTORY_STATUS_IMPOSSIBLE = 7

    def __init__(self):
        self.header = Header()
        self.position = Point()
        self.velocity = Vector3()
        self.acceleration = Vector3()
        self.jerk = Vector3()
        self.yaw = 0.0
        self.yaw_dot = 0.0
        self.kx = [0.0] * 3
        self.kv = [0.0] * 3
        self.trajectory_id = 0
        self.trajectory_flag = 0


class ModelStates:
    def __init__(self):
        self.name = []
        self.pose = []


def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class RecordingPublisher:
    def __init__(self, topic, message_type, **kwargs):
        self.topic = topic
        self.message_type = message_type
        self.published = []

    def publish(self, message):
        self.published.append(message)

    def get_num_connections(self):
        return 1


class _FakeTimeFactory:
    def __init__(self, rospy_handle):
        self._rospy = rospy_handle
        self.from_sec = FakeTime

    def now(self):
        return FakeTime(self._rospy.now_seconds)


class _FakeDuration:
    def __init__(self, seconds):
        self.seconds = seconds


class FakeRospy(types.ModuleType):
    def __init__(self):
        super().__init__("rospy")
        self.parameters = {}
        self.publishers = []
        self.subscribers = []
        self.timers = []
        self.warnings = []
        self.now_seconds = 100.0
        self.Time = _FakeTimeFactory(self)
        self.Duration = _FakeDuration
        self.Publisher = self._publisher
        self.Subscriber = self._subscriber
        self.Timer = self._timer
        self.logwarn_throttle = self._warn
        self.loginfo = lambda *args, **kwargs: None
        self.logerr = lambda *args, **kwargs: None
        self.ROSInitException = RuntimeError

    class _Duration:
        def __init__(self, seconds):
            self.seconds = seconds

    _MISSING = object()

    def get_param(self, name, default=_MISSING):
        if name in self.parameters:
            return self.parameters[name]
        if default is FakeRospy._MISSING:
            raise KeyError(name)
        return default

    def _publisher(self, topic, message_type, **kwargs):
        publisher = RecordingPublisher(topic, message_type, **kwargs)
        self.publishers.append(publisher)
        return publisher

    def _subscriber(self, topic, message_type, callback, **kwargs):
        record = types.SimpleNamespace(
            topic=topic, message_type=message_type, callback=callback
        )
        self.subscribers.append(record)
        return record

    def _timer(self, duration, callback, **kwargs):
        record = types.SimpleNamespace(period=duration.seconds, callback=callback)
        self.timers.append(record)
        return record

    def _warn(self, *args, **kwargs):
        self.warnings.append(args)

    def Time_now(self):
        return FakeTime(self.now_seconds)


def install_fake_ros():
    """Inject fake modules and return the fake rospy handle."""
    fake_rospy = FakeRospy()
    modules = {
        "rospy": fake_rospy,
        "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": types.ModuleType("nav_msgs.msg"),
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
        "quadrotor_msgs": types.ModuleType("quadrotor_msgs"),
        "quadrotor_msgs.msg": types.ModuleType("quadrotor_msgs.msg"),
        "gazebo_msgs": types.ModuleType("gazebo_msgs"),
        "gazebo_msgs.msg": types.ModuleType("gazebo_msgs.msg"),
        "tf": types.ModuleType("tf"),
        "tf.transformations": types.ModuleType("tf.transformations"),
    }
    modules["nav_msgs.msg"].Odometry = Odometry
    modules["geometry_msgs.msg"].PoseWithCovarianceStamped = PoseWithCovarianceStamped
    modules["geometry_msgs.msg"].PointStamped = PointStamped
    modules["std_msgs.msg"].String = String
    modules["quadrotor_msgs.msg"].PositionCommand = PositionCommand
    modules["gazebo_msgs.msg"].ModelStates = ModelStates
    modules["tf.transformations"].euler_from_quaternion = euler_from_quaternion
    modules["tf.transformations"].quaternion_from_euler = quaternion_from_euler
    saved = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    return fake_rospy, saved


def restore_ros(saved):
    for name, module in saved.items():
        if module is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


def load_script_class(script_name, class_name):
    spec = importlib.util.spec_from_file_location(
        "script_under_test_" + script_name.replace(".", "_"), SCRIPTS / script_name
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, class_name)
