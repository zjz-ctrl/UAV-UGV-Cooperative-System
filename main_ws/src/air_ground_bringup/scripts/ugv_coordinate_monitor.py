#!/usr/bin/env python3
"""Continuously print the UGV pose in the registered common frame."""

import math
import threading

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from air_ground_coordinate_transform.msg import RegistrationUpdate


class UgvCoordinateMonitor:
    def __init__(self):
        self.phase = "WAIT"
        self.registered = False
        self.reference = None
        self.registration_lock = threading.Lock()
        self.registration_revision = 0
        self.previous_registration = None
        self.last_print = rospy.Time(0)
        self.print_rate = float(rospy.get_param("~print_rate", 2.0))
        self.origin_frame = rospy.get_param("~origin_frame", "air_ground_origin")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(
            rospy.get_param("~mission_phase_topic", "/air_ground/mission_phase"),
            String, self.phase_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~registration_topic", "/air_ground/registration/frozen"),
            Bool, self.registration_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param(
                "~registration_accepted_update_topic",
                "/air_ground/registration/accepted_update",
            ),
            RegistrationUpdate, self.accepted_update_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/ugv_0/odom"),
            Odometry, self.odom_callback, queue_size=20)

    def phase_callback(self, message):
        self.phase = message.data

    def registration_callback(self, message):
        if message.data and not self.registered:
            self.reference = None
            self.last_print = rospy.Time(0)
        self.registered = message.data

    def accepted_update_callback(self, message):
        revision = int(message.revision)
        pose = message.pose.pose
        estimate = (
            pose.position.x,
            pose.position.y,
            self.yaw_from_quaternion(pose.orientation),
            tuple(message.pose.covariance),
        )
        with self.registration_lock:
            if revision <= self.registration_revision or revision <= 0:
                return
            self._commit_registration(revision, estimate)

    def _commit_registration(self, revision, estimate):
        x, y, yaw, covariance = estimate
        sigma_x = math.sqrt(max(0.0, covariance[0]))
        sigma_y = math.sqrt(max(0.0, covariance[7]))
        sigma_yaw_deg = math.degrees(math.sqrt(max(0.0, covariance[35])))
        if self.previous_registration is None:
            dx = dy = dyaw_deg = 0.0
        else:
            dx = x - self.previous_registration[0]
            dy = y - self.previous_registration[1]
            dyaw_deg = math.degrees(
                self.normalize_angle(yaw - self.previous_registration[2])
            )
        rospy.loginfo(
            "Registration revision=%d sigma_x=%.4f sigma_y=%.4f "
            "sigma_yaw_deg=%.3f registration_delta=(%.4f, %.4f, %.3f deg)",
            revision,
            sigma_x,
            sigma_y,
            sigma_yaw_deg,
            dx,
            dy,
            dyaw_deg,
        )
        self.previous_registration = (x, y, yaw)
        self.registration_revision = revision

    @staticmethod
    def yaw_from_quaternion(quaternion):
        sin_yaw = 2.0 * (quaternion.w * quaternion.z +
                         quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, message):
        if not self.registered:
            return
        now = rospy.Time.now()
        if self.print_rate > 0.0:
            period = 1.0 / self.print_rate
            if not self.last_print.is_zero() and (now - self.last_print).to_sec() < period:
                return
        self.last_print = now

        source_frame = message.header.frame_id or "ugv_0/odom"
        try:
            transform = self.tf_buffer.lookup_transform(
                self.origin_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(2.0, "Waiting for registered UGV transform: %s", error)
            return

        position = message.pose.pose.position
        translation = transform.transform.translation
        transform_yaw = self.yaw_from_quaternion(transform.transform.rotation)
        cos_yaw, sin_yaw = math.cos(transform_yaw), math.sin(transform_yaw)
        x = translation.x + cos_yaw * position.x - sin_yaw * position.y
        y = translation.y + sin_yaw * position.x + cos_yaw * position.y
        z = translation.z + position.z
        yaw = self.normalize_angle(
            transform_yaw + self.yaw_from_quaternion(message.pose.pose.orientation))

        if self.reference is None:
            self.reference = (x, y, z)
        dx, dy, dz = (x - self.reference[0], y - self.reference[1],
                      z - self.reference[2])
        rospy.loginfo(
            "UGV merged coordinate: t=%.3f phase=%s frame=%s "
            "x=%.3f y=%.3f z=%.3f yaw=%.2f deg "
            "travel_delta=(%.3f, %.3f, %.3f)",
            message.header.stamp.to_sec(), self.phase, self.origin_frame,
            x, y, z, math.degrees(yaw), dx, dy, dz)


if __name__ == "__main__":
    rospy.init_node("ugv_coordinate_monitor")
    UgvCoordinateMonitor()
    rospy.spin()
