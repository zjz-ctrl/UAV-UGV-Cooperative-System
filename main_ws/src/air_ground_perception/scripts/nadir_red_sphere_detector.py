#!/usr/bin/env python3
"""Detect a red sphere in the nadir RGB image and publish its optical ray."""

import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


class NadirRedSphereDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.info = None
        self.minimum_area = float(rospy.get_param("~minimum_area", 80.0))
        self.minimum_circularity = float(rospy.get_param("~minimum_circularity", 0.60))
        self.ray_pub = rospy.Publisher(
            rospy.get_param("~ray_topic", "/air_ground/red_sphere/nadir/ray"),
            PointStamped, queue_size=2)
        self.valid_pub = rospy.Publisher(
            rospy.get_param("~valid_topic", "/air_ground/red_sphere/nadir/valid"),
            Bool, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~camera_info_topic", "/iris_0/nadir_camera/camera_info"),
            CameraInfo, self.info_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~image_topic", "/iris_0/nadir_camera/image_raw"),
            Image, self.image_callback, queue_size=1, buff_size=2 ** 24)

    def info_callback(self, message):
        if message.K[0] > 0.0 and message.K[4] > 0.0:
            self.info = message

    def reject(self):
        self.valid_pub.publish(False)

    def image_callback(self, message):
        if self.info is None:
            self.reject()
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            rospy.logwarn_throttle(2.0, "Nadir red-sphere image conversion failed: %s", error)
            self.reject()
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = (cv2.inRange(hsv, (0, 100, 60), (10, 255, 255)) |
                cv2.inRange(hsv, (170, 100, 60), (179, 255, 255)))
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.reject()
            return

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        circularity = 0.0 if perimeter <= 0.0 else 4.0 * math.pi * area / (perimeter ** 2)
        moments = cv2.moments(contour)
        if (area < self.minimum_area or circularity < self.minimum_circularity or
                abs(moments["m00"]) < 1e-6):
            self.reject()
            return

        u = moments["m10"] / moments["m00"]
        v = moments["m01"] / moments["m00"]
        intrinsics = self.info.K
        ray = PointStamped()
        ray.header = message.header
        ray.header.frame_id = self.info.header.frame_id or message.header.frame_id
        ray.point.x = (u - intrinsics[2]) / intrinsics[0]
        ray.point.y = (v - intrinsics[5]) / intrinsics[4]
        ray.point.z = 1.0
        self.ray_pub.publish(ray)
        self.valid_pub.publish(True)


if __name__ == "__main__":
    rospy.init_node("nadir_red_sphere_detector")
    NadirRedSphereDetector()
    rospy.spin()
