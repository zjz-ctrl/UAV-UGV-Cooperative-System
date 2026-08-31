#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from quadrotor_msgs.msg import PositionCommand
import tf2_ros
import tf2_geometry_msgs
import message_filters

class RingGoalFromD435:
    def __init__(self):
        rospy.loginfo("RingGoalFromD435 init...")

        # ---------- Params ----------
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/iris_D435i/realsense/depth_camera/color/camera_info")
        self.color_topic = rospy.get_param("~color_topic", "/iris_D435i/realsense/depth_camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/iris_D435i/realsense/depth_camera/depth/image_raw")

        self.target_frame = rospy.get_param("~target_frame", "map")
        self.publish_topic = rospy.get_param("~position_cmd_topic", "/position_cmd")

        self.depth_scale = rospy.get_param("~depth_scale", 1.0)
        self.min_depth = rospy.get_param("~min_depth", 0.2)
        self.max_depth = rospy.get_param("~max_depth", 10.0)

        self.standoff = rospy.get_param("~standoff", 1.0)

        self.dp = rospy.get_param("~dp", 1.2)
        self.minDist = rospy.get_param("~minDist", 50)
        self.param1 = rospy.get_param("~param1", 120)
        self.param2 = rospy.get_param("~param2", 35)
        self.minRadius = rospy.get_param("~minRadius", 20)
        self.maxRadius = rospy.get_param("~maxRadius", 120)

        self.debug_view = rospy.get_param("~debug_view", True)

        # ---------- State ----------
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.cam_frame = None

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ---------- Pub ----------
        self.cmd_pub = rospy.Publisher(self.publish_topic, PositionCommand, queue_size=1)

        # ---------- Subs ----------
        self.info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_cb, queue_size=1)

        color_sub = message_filters.Subscriber(self.color_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)

        # 同步彩色图像和深度图像
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.synced_cb)

        rospy.loginfo("RingGoalFromD435 ready.")
        rospy.loginfo("color: %s", self.color_topic)
        rospy.loginfo("depth: %s", self.depth_topic)
        rospy.loginfo("camera_info: %s", self.camera_info_topic)
        rospy.loginfo("target_frame: %s", self.target_frame)

        # ---------- 发布 TF ----------
        self.publish_static_tf()

    def publish_static_tf(self):
        rospy.loginfo("Publishing static transform from depth_camera_base to %s", self.target_frame)
        rospy.sleep(1)  
        self.static_transform_publisher()

    def static_transform_publisher(self):
        static_tf_publisher = rospy.Publisher('/static_transform', tf2_ros.TransformStamped, queue_size=1)

        transform = tf2_ros.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "depth_camera_base"
        transform.child_frame_id = self.target_frame

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        static_tf_publisher.publish(transform)

    def camera_info_cb(self, msg: CameraInfo):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.cam_frame = msg.header.frame_id if msg.header.frame_id else msg.header.frame_id

    def detect_circle_center(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 1.5)

        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT,
            dp=self.dp, minDist=self.minDist,
            param1=self.param1, param2=self.param2,
            minRadius=self.minRadius, maxRadius=self.maxRadius
        )

        if circles is None:
            return None, None

        circles = np.round(circles[0, :]).astype(np.int32)
        best = max(circles, key=lambda c: c[2])
        u, v, r = int(best[0]), int(best[1]), int(best[2])
        return (u, v), r

    def depth_at(self, depth_img, u, v, r):
        h, w = depth_img.shape[:2]
        depth_img = cv2.GaussianBlur(depth_img, (5, 5), 0)
        num_samples = 16
        samples = []

        for i in range(num_samples):
            angle = 2 * np.pi * i / num_samples
            sample_u = int(u + r * np.cos(angle))
            sample_v = int(v + r * np.sin(angle))

            if 0 <= sample_u < w and 0 <= sample_v < h:
                depth = depth_img[sample_v, sample_u] * self.depth_scale
                if np.isfinite(depth) and self.min_depth < depth < self.max_depth:
                    samples.append(depth)

        if len(samples) < 5:
            rospy.logwarn("Too few valid depth points on the ring at (%d, %d)", u, v)
            return None

        return np.mean(samples)

    def pixel_to_cam(self, u, v, z):
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    def publish_position_cmd(self, xyz_world, stamp):
        msg = PositionCommand()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame

        msg.position.x = xyz_world[0]
        msg.position.y = xyz_world[1]
        msg.position.z = xyz_world[2]

        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0
        msg.jerk.x = 0.0
        msg.jerk.y = 0.0
        msg.jerk.z = 0.0

        msg.yaw = 0.0
        msg.yaw_dot = 0.0

        msg.trajectory_id = 1
        msg.trajectory_flag = 1

        rospy.loginfo("Publishing position_cmd: (%f, %f, %f)", xyz_world[0], xyz_world[1], xyz_world[2])
        self.cmd_pub.publish(msg)

    def synced_cb(self, color_msg: Image, depth_msg: Image):
        if self.fx is None:
            rospy.logwarn_throttle(2.0, "No camera_info yet, waiting...")
            return

        cam_frame = color_msg.header.frame_id if color_msg.header.frame_id else self.cam_frame
        if not cam_frame:
            rospy.logwarn_throttle(2.0, "camera frame_id empty, cannot TF.")
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        except Exception as e:
            rospy.logerr_throttle(2.0, "color cv_bridge error: %s", str(e))
            return

        try:
            if depth_msg.encoding == "16UC1":
                depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1").astype(np.float32)
                depth_scale = rospy.get_param("~depth_scale_16u", 0.001)
                depth = depth * depth_scale
            else:
                depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1").astype(np.float32) * self.depth_scale
        except Exception as e:
            rospy.logerr_throttle(2.0, "depth cv_bridge error: %s", str(e))
            return

        center, r = self.detect_circle_center(bgr)
        if center is None:
            if self.debug_view:
                cv2.imshow("ring_detect", bgr)
                cv2.waitKey(1)
            return

        u, v = center
        rospy.loginfo("Detected circle at (%d, %d)", u, v)

        z = self.depth_at(depth, u, v, r)
        if z is None:
            rospy.logwarn_throttle(1.0, "Circle detected but depth invalid at (%d,%d).", u, v)
            if self.debug_view:
                cv2.circle(bgr, (u, v), 5, (0, 0, 255), -1)
                cv2.circle(bgr, (u, v), r, (0, 255, 0), 2)
                cv2.imshow("ring_detect", bgr)
                cv2.waitKey(1)
            return

        x_cam, y_cam, z_cam = self.pixel_to_cam(u, v, z)
        rospy.loginfo("Depth at circle center: %.2f m", z_cam)

        z_goal_cam = max(self.min_depth, z_cam - self.standoff)
        goal_cam = np.array([x_cam, y_cam, z_goal_cam], dtype=np.float32)

        ps = PointStamped()
        ps.header.stamp = color_msg.header.stamp
        ps.header.frame_id = cam_frame
        ps.point.x = float(goal_cam[0])
        ps.point.y = float(goal_cam[1])
        ps.point.z = float(goal_cam[2])

        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame, ps.header.frame_id,
                ps.header.stamp, rospy.Duration(0.05)
            )
            pw = tf2_geometry_msgs.do_transform_point(ps, trans)
            xyz_world = (pw.point.x, pw.point.y, pw.point.z)

            self.publish_position_cmd(xyz_world, color_msg.header.stamp)

            rospy.loginfo_throttle(
                0.5,
                "ring (u,v,r)=(%d,%d,%d) depth=%.2f  goal_world=(%.2f,%.2f,%.2f)",
                u, v, r, z, xyz_world[0], xyz_world[1], xyz_world[2]
            )

        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF failed: %s", str(e))

        if self.debug_view:
            cv2.circle(bgr, (u, v), 5, (0, 0, 255), -1)
            cv2.circle(bgr, (u, v), r, (0, 255, 0), 2)
            cv2.putText(bgr, f"z={z:.2f}m", (u + 10, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("ring_detect", bgr)
            cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("ring_goal_from_d435", anonymous=False)
    node = RingGoalFromD435()
    rospy.spin()
