#!/usr/bin/env python3
"""Drive the UGV through a bounded demonstration route using only its own odometry."""

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class UgvPatrol:
    def __init__(self):
        self.odom = None
        self.enabled = False
        self.origin = None
        self.index = 0
        self.speed = rospy.get_param("~speed", 0.35)
        self.turn_speed = rospy.get_param("~turn_speed", 0.65)
        self.waypoint_radius = rospy.get_param("~waypoint_radius", 0.20)
        self.loop_route = rospy.get_param("~loop_route", True)
        # A compact loop lets the follower maintain a forward camera view of the UGV.
        self.relative_route = rospy.get_param("~route", [[0.0, 0.0], [0.0, 1.0], [1.0, 2.0], [2.0, 2.0], [2.0, 0.0], [1.0, -1.0], [0.0, 0.0], [-1.0, -1.0], [-2.0, 0.0], [-2.0, 2.0], [-1.0, 3.0], [0.0, 2.0]])
        self.command_pub = rospy.Publisher("/ugv_0/cmd_vel", Twist, queue_size=1)
        self.complete_pub = rospy.Publisher("/air_ground/patrol_complete", Bool, queue_size=1, latch=True)
        self.markers_pub = rospy.Publisher("/air_ground/patrol_waypoints", MarkerArray, queue_size=1, latch=True)
        rospy.Subscriber("/ugv_0/odom", Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber("/air_ground/start_patrol", Bool, self.enable_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.control)

    def odom_callback(self, message):
        self.odom = message

    def enable_callback(self, message):
        if message.data and not self.enabled and self.odom is not None:
            self.enabled = True
            position = self.odom.pose.pose.position
            self.origin = (position.x, position.y)
            self.publish_waypoints()
            rospy.loginfo("UGV patrol started")

    def publish_waypoints(self):
        markers = MarkerArray()
        for index, waypoint in enumerate(self.relative_route):
            marker = Marker()
            marker.header.frame_id = "ugv_0/odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "patrol_waypoints"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.origin[0] + waypoint[0]
            marker.pose.position.y = self.origin[1] + waypoint[1]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.20
            marker.color.g = 1.0
            marker.color.a = 0.9
            markers.markers.append(marker)
        self.markers_pub.publish(markers)

    def control(self, _event):
        command = Twist()
        if not self.enabled or self.odom is None or self.origin is None:
            self.command_pub.publish(command)
            return
        if self.index >= len(self.relative_route):
            if self.loop_route:
                self.index = 0
            else:
                self.command_pub.publish(command)
                self.complete_pub.publish(Bool(data=True))
                return
        point = self.relative_route[self.index]
        target_x = self.origin[0] + point[0]
        target_y = self.origin[1] + point[1]
        position = self.odom.pose.pose.position
        quaternion = self.odom.pose.pose.orientation
        yaw = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))[2]
        dx, dy = target_x - position.x, target_y - position.y
        distance = math.hypot(dx, dy)
        if distance < self.waypoint_radius:
            self.index += 1
            return
        heading_error = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
        command.angular.z = clamp(1.5 * heading_error, -self.turn_speed, self.turn_speed)
        command.linear.x = self.speed * max(0.0, 1.0 - abs(heading_error) / 1.2)
        self.command_pub.publish(command)


if __name__ == "__main__":
    rospy.init_node("ugv_patrol")
    UgvPatrol()
    rospy.spin()
