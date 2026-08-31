#!/usr/bin/env python3
"""Single-owner UGV point controller for goals in the frozen task frame."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def angle_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class Controller:
    def __init__(self):
        self.odom = None
        self.goal = None
        self.arrived = False
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 0.025))
        self.hold_tolerance = float(rospy.get_param("~hold_tolerance", 0.05))
        self.yaw_tolerance = float(rospy.get_param("~yaw_tolerance", 0.05))
        self.maximum_odom_age = float(rospy.get_param("~maximum_odom_age", 0.5))
        self.odom_topic = rospy.get_param("~odom_topic", "/ugv_0/odom")
        self.odom_frame = rospy.get_param("~odom_frame", "ugv_0/odom")
        self.last_odom_received = None
        self.waiting_for_odom = False

        self.command_pub = rospy.Publisher("/ugv_0/cmd_vel", Twist, queue_size=1)
        self.goal_odom_pub = rospy.Publisher("/air_ground/ugv_goal_odom", PoseStamped, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/air_ground/ugv/status", String, queue_size=1, latch=True)
        self.arrived_pub = rospy.Publisher("/air_ground/ugv/arrived", Bool, queue_size=1, latch=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/air_ground/ugv_goal", PoseStamped, self.goal_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)
        self.status_pub.publish("IDLE")

    def odom_callback(self, message):
        if message.header.frame_id != self.odom_frame:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring UGV odometry frame '%s'; expected '%s'",
                message.header.frame_id,
                self.odom_frame,
            )
            self.odom = None
            self.last_odom_received = None
            return
        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(2.0, "Ignoring nonfinite UGV odometry")
            self.odom = None
            self.last_odom_received = None
            return
        self.odom = message
        self.last_odom_received = rospy.Time.now()
        if self.waiting_for_odom and self.goal is not None:
            self.waiting_for_odom = False
            self.status_pub.publish("NAVIGATING")

    def goal_callback(self, message):
        source_orientation = message.pose.orientation
        source_yaw = euler_from_quaternion((source_orientation.x, source_orientation.y,
                                            source_orientation.z, source_orientation.w))[2]
        if message.header.frame_id == self.odom_frame:
            self.goal = (
                message.pose.position.x,
                message.pose.position.y,
                angle_error(source_yaw, 0.0),
            )
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    message.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.2),
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as error:
                rospy.logwarn("Rejecting UGV goal: %s", error)
                return

            rotation = transform.transform.rotation
            rotation_matrix = quaternion_matrix(
                (rotation.x, rotation.y, rotation.z, rotation.w))
            point = rotation_matrix.dot([
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
                1.0,
            ])
            translation = transform.transform.translation
            transform_yaw = euler_from_quaternion(
                (rotation.x, rotation.y, rotation.z, rotation.w))[2]
            self.goal = (
                point[0] + translation.x,
                point[1] + translation.y,
                angle_error(transform_yaw + source_yaw, 0.0),
            )
        self.arrived = False
        self.waiting_for_odom = False
        self.arrived_pub.publish(False)
        self.status_pub.publish("NAVIGATING")

        goal_odom = PoseStamped()
        goal_odom.header.stamp = rospy.Time.now()
        goal_odom.header.frame_id = self.odom_frame
        goal_odom.pose.position.x, goal_odom.pose.position.y = self.goal[:2]
        quaternion = quaternion_from_euler(0.0, 0.0, self.goal[2])
        goal_odom.pose.orientation.x, goal_odom.pose.orientation.y = quaternion[:2]
        goal_odom.pose.orientation.z, goal_odom.pose.orientation.w = quaternion[2:]
        self.goal_odom_pub.publish(goal_odom)
        rospy.loginfo("Accepted UGV odom goal (%.3f, %.3f, yaw %.3f)", *self.goal)

    def tick(self, _event):
        command = Twist()
        if self.goal is None:
            self.command_pub.publish(command)
            return
        odom_age = (
            float("inf")
            if self.last_odom_received is None
            else (rospy.Time.now() - self.last_odom_received).to_sec()
        )
        if self.odom is None or odom_age > self.maximum_odom_age:
            if not self.waiting_for_odom:
                self.waiting_for_odom = True
                self.status_pub.publish("WAITING_ODOMETRY")
            if self.arrived:
                self.arrived = False
                self.arrived_pub.publish(False)
            self.command_pub.publish(command)
            return

        pose = self.odom.pose.pose
        yaw = euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                     pose.orientation.z, pose.orientation.w))[2]
        dx, dy = self.goal[0] - pose.position.x, self.goal[1] - pose.position.y
        distance = math.hypot(dx, dy)

        if self.arrived and distance <= self.hold_tolerance:
            yaw_error = angle_error(self.goal[2], yaw)
            if abs(yaw_error) > self.yaw_tolerance:
                command.angular.z = clamp(1.5 * yaw_error, -0.45, 0.45)
            self.command_pub.publish(command)
            return
        if self.arrived:
            self.arrived = False
            self.arrived_pub.publish(False)
            self.status_pub.publish("NAVIGATING")

        if distance <= self.position_tolerance:
            yaw_error = angle_error(self.goal[2], yaw)
            if abs(yaw_error) > self.yaw_tolerance:
                command.angular.z = clamp(1.5 * yaw_error, -0.45, 0.45)
                self.status_pub.publish("ALIGNING")
                self.command_pub.publish(command)
                return
            self.arrived = True
            self.command_pub.publish(command)
            self.arrived_pub.publish(True)
            self.status_pub.publish("ARRIVED")
            return

        desired_yaw = math.atan2(dy, dx)
        heading_error = angle_error(desired_yaw, yaw)
        command.angular.z = clamp(1.5 * heading_error, -0.65, 0.65)
        if abs(heading_error) <= 0.7:
            command.linear.x = min(0.35, 0.55 * distance) * max(0.0, math.cos(heading_error))
        self.command_pub.publish(command)


if __name__ == "__main__":
    rospy.init_node("ugv_goal_controller")
    Controller()
    rospy.spin()
