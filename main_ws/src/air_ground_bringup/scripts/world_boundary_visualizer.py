#!/usr/bin/env python3
"""Publish the static Gazebo arena walls as RViz markers."""

import math

import rospy
from visualization_msgs.msg import Marker, MarkerArray


class WorldBoundaryVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "iris_0/odom")
        self.translation = rospy.get_param("~world_to_uav_odom_translation", [6.0, 0.0, 0.0])
        self.walls = rospy.get_param("~walls", [])
        self.publisher = rospy.Publisher("/air_ground/world_boundaries", MarkerArray, queue_size=1, latch=True)
        self.publish()
        rospy.Timer(rospy.Duration(1.0), self.publish)

    def publish(self, _event=None):
        markers = MarkerArray()
        for index, wall in enumerate(self.walls):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.frame_id
            marker.ns = "gazebo_world_boundaries"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(wall["x"]) + float(self.translation[0])
            marker.pose.position.y = float(wall["y"]) + float(self.translation[1])
            marker.pose.position.z = float(wall["z"]) + float(self.translation[2])
            half_yaw = float(wall["yaw"]) * 0.5
            marker.pose.orientation.z = math.sin(half_yaw)
            marker.pose.orientation.w = math.cos(half_yaw)
            marker.scale.x = float(wall["size_x"])
            marker.scale.y = float(wall["size_y"])
            marker.scale.z = float(wall["size_z"])
            marker.color.r = 0.95
            marker.color.g = 0.35
            marker.color.b = 0.10
            marker.color.a = 0.45
            markers.markers.append(marker)
        self.publisher.publish(markers)


if __name__ == "__main__":
    rospy.init_node("world_boundary_visualizer")
    WorldBoundaryVisualizer()
    rospy.spin()
