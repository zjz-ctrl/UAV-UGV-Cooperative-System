#!/usr/bin/env python3
"""Fly a CXR-controlled UAV using only vision-derived UGV target estimates."""

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class UavFollowMission:
    def __init__(self):
        self.state = State()
        self.odom = None
        self.target = None
        self.target_valid = False
        self.patrol_complete = False
        self.phase = "WAIT_SYSTEM"
        self.phase_started = rospy.Time.now()
        self.start_pose = None
        self.last_command = None
        self.patrol_started = False
        self.land_requested = False
        self.takeoff_height = rospy.get_param("~takeoff_height", 1.5)
        self.follow_back = rospy.get_param("~follow_back", 0.4)
        self.follow_height = rospy.get_param("~follow_height", 1.5)
        self.target_timeout = rospy.get_param("~target_timeout", 0.45)
        self.search_timeout = rospy.get_param("~search_timeout", 20.0)
        self.lost_timeout = rospy.get_param("~lost_timeout", 3.0)
        self.takeoff_duration = rospy.get_param("~takeoff_duration", 6.0)
        self.rate_hz = rospy.get_param("~rate", 30.0)
        self.command_pub = rospy.Publisher("/iris_0/position_cmd", PositionCommand, queue_size=10)
        self.patrol_pub = rospy.Publisher("/air_ground/start_patrol", Bool, queue_size=1, latch=True)
        self.phase_pub = rospy.Publisher("/air_ground/demo_phase", String, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/air_ground/follow_target_marker", Marker, queue_size=1)
        self.param_set = rospy.ServiceProxy("/iris_0/mavros/param/set", ParamSet)
        self.param_get = rospy.ServiceProxy("/iris_0/mavros/param/get", ParamGet)
        self.set_mode = rospy.ServiceProxy("/iris_0/mavros/set_mode", SetMode)
        self.arm = rospy.ServiceProxy("/iris_0/mavros/cmd/arming", CommandBool)
        rospy.Subscriber("/iris_0/mavros/state", State, self.state_callback, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/local_position/odom", Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber("/air_ground/relative_target", PoseWithCovarianceStamped, self.target_callback, queue_size=20)
        rospy.Subscriber("/air_ground/relative_target_valid", Bool, self.target_valid_callback, queue_size=1)
        rospy.Subscriber("/air_ground/patrol_complete", Bool, self.patrol_complete_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.tick)

    def state_callback(self, message):
        self.state = message

    def odom_callback(self, message):
        self.odom = message

    def target_callback(self, message):
        self.target = message

    def target_valid_callback(self, message):
        self.target_valid = message.data

    def patrol_complete_callback(self, message):
        self.patrol_complete = message.data

    def set_phase(self, phase):
        if phase != self.phase:
            rospy.loginfo("Air-ground demo phase: %s", phase)
            self.phase = phase
            self.phase_started = rospy.Time.now()
            self.phase_pub.publish(String(data=phase))

    def target_is_fresh(self):
        if not self.target_valid or self.target is None:
            return False
        stamp = self.target.header.stamp
        return not stamp.is_zero() and (rospy.Time.now() - stamp).to_sec() <= self.target_timeout

    def publish_command(self, x, y, z, yaw):
        command = PositionCommand()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = "iris_0/odom"
        command.position.x = x
        command.position.y = y
        command.position.z = z
        command.yaw = yaw
        command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.command_pub.publish(command)
        self.last_command = (x, y, z, yaw)

    def publish_hold(self):
        if self.last_command is not None:
            self.publish_command(*self.last_command)
        elif self.odom is not None:
            position = self.odom.pose.pose.position
            self.publish_command(position.x, position.y, position.z, 0.0)

    def configure_px4(self):
        try:
            if not self.param_get(param_id="COM_RCL_EXCEPT").success:
                return False
            result = self.param_set(param_id="COM_RCL_EXCEPT", value=ParamValue(integer=4, real=0.0))
            return result.success
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn_throttle(2.0, "Cannot configure PX4 Offboard exception: %s", error)
            return False

    def request_mode(self, mode):
        try:
            self.set_mode(base_mode=0, custom_mode=mode)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "Cannot request PX4 mode %s: %s", mode, error)

    def request_arm(self, arm):
        try:
            self.arm(arm)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "Cannot change PX4 arm state: %s", error)

    def desired_follow_command(self):
        target = self.target.pose.pose.position
        orientation = self.target.pose.pose.orientation
        ugv_yaw = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2]
        return target.x - self.follow_back * math.cos(ugv_yaw), target.y - self.follow_back * math.sin(ugv_yaw), target.z + self.follow_height, ugv_yaw

    def publish_target_marker(self, command):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "iris_0/odom"
        marker.ns = "follow_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = command[:3]
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.30
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        self.marker_pub.publish(marker)

    def tick(self, _event):
        if self.phase == "WAIT_SYSTEM":
            if self.odom is None or not self.state.connected:
                return
            if not self.configure_px4():
                return
            position = self.odom.pose.pose.position
            self.start_pose = (position.x, position.y, position.z)
            self.last_command = (position.x, position.y, max(position.z, 0.15), 0.0)
            self.set_phase("PRESTREAM")
            return

        self.publish_hold()
        elapsed = (rospy.Time.now() - self.phase_started).to_sec()
        if self.phase == "PRESTREAM":
            if elapsed >= 2.0:
                self.set_phase("REQUEST_OFFBOARD")
            return
        if self.phase == "REQUEST_OFFBOARD":
            if self.state.mode == "OFFBOARD":
                self.set_phase("REQUEST_ARM")
            elif elapsed >= 1.0:
                self.request_mode("OFFBOARD")
                self.phase_started = rospy.Time.now()
            return
        if self.phase == "REQUEST_ARM":
            if self.state.armed:
                self.set_phase("TAKEOFF")
            elif elapsed >= 1.0:
                self.request_arm(True)
                self.phase_started = rospy.Time.now()
            return
        if self.phase == "TAKEOFF":
            ratio = clamp(elapsed / self.takeoff_duration, 0.0, 1.0)
            smooth = ratio * ratio * (3.0 - 2.0 * ratio)
            x, y, start_z = self.start_pose
            self.publish_command(x, y, start_z + (self.takeoff_height - start_z) * smooth, 0.0)
            if ratio >= 1.0:
                self.set_phase("SEARCH")
            return
        if self.phase == "SEARCH":
            if self.target_is_fresh():
                self.patrol_pub.publish(Bool(data=True))
                self.patrol_started = True
                self.set_phase("FOLLOW")
            elif elapsed > self.search_timeout:
                rospy.logwarn("UGV was not visually acquired; returning to launch point")
                self.set_phase("RETURN")
            return
        if self.phase == "FOLLOW":
            if self.patrol_complete:
                self.set_phase("RETURN")
                return
            if self.target_is_fresh():
                command = self.desired_follow_command()
                self.publish_command(*command)
                self.publish_target_marker(command)
                self.phase_started = rospy.Time.now()
            elif elapsed > self.lost_timeout:
                rospy.logwarn("Visual UGV target lost; returning to launch point")
                self.set_phase("RETURN")
            return
        if self.phase == "RETURN":
            x, y, _ = self.start_pose
            self.publish_command(x, y, self.takeoff_height, 0.0)
            position = self.odom.pose.pose.position
            if math.hypot(position.x - x, position.y - y) < 0.25 and elapsed > 2.0:
                self.set_phase("LAND")
            return
        if self.phase == "LAND":
            if not self.land_requested:
                self.request_mode("AUTO.LAND")
                self.land_requested = True
            if not self.state.armed:
                self.set_phase("COMPLETE")


if __name__ == "__main__":
    rospy.init_node("uav_follow_mission")
    UavFollowMission()
    rospy.spin()
