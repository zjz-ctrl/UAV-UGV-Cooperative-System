#!/usr/bin/env python3
"""Detect a known-radius red sphere from the UAV's forward aligned RGB-D stream."""
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


class RedSphereDetector:
    def __init__(self):
        self.bridge, self.info, self.depth = CvBridge(), None, None
        self.radius = float(rospy.get_param("~sphere_radius", 0.25))
        self.minimum_area = float(rospy.get_param("~minimum_area", 50.0))
        self.minimum_circularity = float(rospy.get_param("~minimum_circularity", 0.60))
        self.minimum_core_points = int(rospy.get_param("~minimum_core_points", 12))
        self.minimum_fit_points = int(rospy.get_param("~minimum_fit_points", 30))
        self.maximum_depth = float(rospy.get_param("~maximum_depth", 20.0))
        self.maximum_fit_error = float(rospy.get_param("~maximum_fit_error", 0.03))
        self.point = rospy.Publisher(
            rospy.get_param("~point_topic", "/air_ground/red_sphere/front/camera_point"),
            PointStamped, queue_size=2)
        self.valid = rospy.Publisher(
            rospy.get_param("~valid_topic", "/air_ground/red_sphere/front/valid"),
            Bool, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~camera_info_topic", "/iris_0/realsense/depth_camera/color/camera_info"),
            CameraInfo, self.info_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~depth_topic", "/iris_0/realsense/depth_camera/depth/image_raw"),
            Image, self.depth_cb, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(rospy.get_param(
            "~image_topic", "/iris_0/realsense/depth_camera/color/image_raw"),
            Image, self.color_cb, queue_size=1, buff_size=2 ** 24)

    def info_cb(self, msg): self.info = msg
    def depth_cb(self, msg): self.depth = msg

    def color_cb(self, msg):
        if self.info is None or self.depth is None or abs((msg.header.stamp - self.depth.header.stamp).to_sec()) > 0.05:
            self.valid.publish(False); return
        try:
            color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough").astype(np.float32)
        except Exception:
            self.valid.publish(False); return
        if self.depth.encoding == "16UC1": depth *= 0.001
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 60), (10, 255, 255)) | cv2.inRange(hsv, (170, 100, 60), (179, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: self.valid.publish(False); return
        contour = max(contours, key=cv2.contourArea); area = cv2.contourArea(contour); perimeter = cv2.arcLength(contour, True)
        if (area < self.minimum_area or perimeter <= 0 or
                4.0 * np.pi * area / (perimeter * perimeter) < self.minimum_circularity):
            self.valid.publish(False); return
        moments = cv2.moments(contour); u, v = moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
        # Initialize the center from the middle of the silhouette, then fit the
        # known-radius sphere to all valid foreground depth points. The fit also
        # handles the perspective shift between silhouette and sphere centers.
        core = np.zeros(mask.shape, np.uint8)
        core_radius = max(3, int(0.25 * np.sqrt(area / np.pi)))
        cv2.circle(core, (int(round(u)), int(round(v))), core_radius, 255, -1)
        values = depth[(core > 0) & (mask > 0) & np.isfinite(depth) &
                       (depth > 0.1) & (depth < self.maximum_depth)]
        if len(values) < self.minimum_core_points: self.valid.publish(False); return
        z = float(np.median(values))
        k = self.info.K; point = PointStamped(); point.header = msg.header; point.header.frame_id = self.info.header.frame_id or msg.header.frame_id
        surface = np.array([(u - k[2]) * z / k[0], (v - k[5]) * z / k[4], z])
        center = surface + self.radius * surface / np.linalg.norm(surface)

        foreground = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(foreground, [contour], -1, 255, -1)
        foreground = cv2.erode(foreground, np.ones((3, 3), np.uint8))
        valid = ((foreground > 0) & np.isfinite(depth) & (depth > 0.1) &
                 (depth < self.maximum_depth))
        rows, columns = np.nonzero(valid)
        if len(rows) < self.minimum_fit_points: self.valid.publish(False); return
        depths = depth[rows, columns]
        points = np.column_stack(((columns - k[2]) * depths / k[0],
                                  (rows - k[5]) * depths / k[4], depths))
        for _ in range(6):
            offsets = center - points
            distances = np.linalg.norm(offsets, axis=1)
            usable = distances > 1e-6
            residuals = distances[usable] - self.radius
            jacobian = offsets[usable] / distances[usable, None]
            try:
                update, _, _, _ = np.linalg.lstsq(jacobian, -residuals, rcond=None)
            except np.linalg.LinAlgError:
                self.valid.publish(False); return
            if not np.all(np.isfinite(update)) or np.linalg.norm(update) > 0.15:
                self.valid.publish(False); return
            center += update
            if np.linalg.norm(update) < 1e-5:
                break
        fit_error = np.sqrt(np.mean((np.linalg.norm(points - center, axis=1) - self.radius) ** 2))
        if not np.isfinite(fit_error) or fit_error > self.maximum_fit_error:
            self.valid.publish(False); return
        point.point.x, point.point.y, point.point.z = center
        self.point.publish(point); self.valid.publish(True)


if __name__ == "__main__":
    rospy.init_node("red_sphere_detector"); RedSphereDetector(); rospy.spin()
