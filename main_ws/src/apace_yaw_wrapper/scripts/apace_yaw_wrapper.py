#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

import message_filters

def quat_to_yaw(qx, qy, qz, qw):
    # yaw from quaternion (ENU)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class ApaceYawWrapper:
    def __init__(self):
        rospy.init_node("apace_yaw_wrapper", anonymous=False)

        # ---- Params ----
        self.cmd_in_topic  = rospy.get_param("~cmd_in",  "/position_cmd_raw")
        self.cmd_out_topic = rospy.get_param("~cmd_out", "/position_cmd")

        self.color_topic   = rospy.get_param("~color", "/iris_D435i/realsense/depth_camera/color/image_raw")
        self.depth_topic   = rospy.get_param("~depth", "/iris_D435i/realsense/depth_camera/depth/image_raw")
        self.cinfo_topic   = rospy.get_param("~camera_info", "/iris_D435i/realsense/depth_camera/color/camera_info")
        self.pose_topic    = rospy.get_param("~pose", "/mavros/local_position/pose")

        self.max_kp        = int(rospy.get_param("~max_keypoints", 250))
        self.depth_min     = float(rospy.get_param("~depth_min_m", 0.3))
        self.depth_max     = float(rospy.get_param("~depth_max_m", 20.0))
        self.yaw_rate_max  = float(rospy.get_param("~yaw_rate_max", 1.0))  # rad/s
        self.lp_alpha      = float(rospy.get_param("~lowpass_alpha", 0.2)) # 0~1
        self.min_valid_pts = int(rospy.get_param("~min_valid_points", 30))

        # image encoding handling
        # if depth is 16UC1 (mm), set ~depth_unit_mm:=true
        self.depth_unit_mm = bool(rospy.get_param("~depth_unit_mm", False))

        # ---- State ----
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.have_cam = False

        self.prev_yaw_cmd = None
        self.prev_time = None

        # ---- ORB ----
        self.orb = cv2.ORB_create(nfeatures=self.max_kp)

        # ---- Subscribers with sync ----
        sub_cmd   = message_filters.Subscriber(self.cmd_in_topic, PositionCommand)
        sub_color = message_filters.Subscriber(self.color_topic, Image)
        sub_depth = message_filters.Subscriber(self.depth_topic, Image)
        sub_pose  = message_filters.Subscriber(self.pose_topic, PoseStamped)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_cmd, sub_color, sub_depth, sub_pose],
            queue_size=20,
            slop=0.05
        )
        self.ts.registerCallback(self.synced_cb)

        self.sub_cinfo = rospy.Subscriber(self.cinfo_topic, CameraInfo, self.cinfo_cb, queue_size=1)

        self.pub = rospy.Publisher(self.cmd_out_topic, PositionCommand, queue_size=10)

        rospy.loginfo("APACE-Yaw Wrapper started.")
        rospy.loginfo("cmd_in: %s  cmd_out: %s", self.cmd_in_topic, self.cmd_out_topic)

    def cinfo_cb(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.have_cam = True

    def depth_at(self, depth_img, u, v):
        h, w = depth_img.shape[:2]
        if u < 0 or u >= w or v < 0 or v >= h:
            return None
        z = depth_img[v, u]
        if np.isnan(z) or z <= 0:
            return None
        if self.depth_unit_mm:
            z = float(z) / 1000.0
        else:
            z = float(z)
        if z < self.depth_min or z > self.depth_max:
            return None
        return z

    def synced_cb(self, cmd_msg: PositionCommand, color_msg: Image, depth_msg: Image, pose_msg: PoseStamped):
        if not self.have_cam:
            rospy.logwarn_throttle(2.0, "Waiting camera_info...")
            return

        # ---- time step ----
        now = cmd_msg.header.stamp if cmd_msg.header.stamp != rospy.Time() else rospy.Time.now()
        if self.prev_time is None:
            dt = 0.02
        else:
            dt = max(1e-3, (now - self.prev_time).to_sec())
        self.prev_time = now

        # ---- current yaw from pose ----
        q = pose_msg.pose.orientation
        yaw_cur = quat_to_yaw(q.x, q.y, q.z, q.w)

        # ---- convert images ----
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr_throttle(2.0, "color cv_bridge error: %s", str(e))
            return

        try:
            # depth could be 32FC1 or 16UC1
            if depth_msg.encoding in ["32FC1", "32FC"]:
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
                self.depth_unit_mm = False
            elif depth_msg.encoding in ["16UC1", "16UC"]:
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
                # may be mm
                # keep parameter decision; if user sets depth_unit_mm=true, we convert
            else:
                # fallback
                depth = self.bridge.imgmsg_to_cv2(depth_msg)
        except Exception as e:
            rospy.logerr_throttle(2.0, "depth cv_bridge error: %s", str(e))
            return

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        # ---- detect keypoints ----
        kps = self.orb.detect(gray, None)
        if kps is None or len(kps) == 0:
            self.publish_cmd(cmd_msg, cmd_msg.yaw, cmd_msg.yaw_dot)
            return

        # sort by response, keep top
        kps = sorted(kps, key=lambda k: k.response, reverse=True)[:self.max_kp]

        # ---- compute "bearing angles" of 3D points in camera frame ----
        # bearing angle beta = atan2(X, Z), where X = (u-cx)*Z/fx
        betas = []
        for kp in kps:
            u = int(round(kp.pt[0]))
            v = int(round(kp.pt[1]))
            z = self.depth_at(depth, u, v)
            if z is None:
                continue
            x = (u - self.cx) * z / self.fx
            beta = math.atan2(x, z)  # left/right bearing
            betas.append(beta)

        if len(betas) < self.min_valid_pts:
            # not enough valid depth points -> keep original yaw (or current yaw)
            self.publish_cmd(cmd_msg, cmd_msg.yaw, cmd_msg.yaw_dot)
            rospy.logwarn_throttle(2.0, "valid depth points too few: %d", len(betas))
            return

        # Robust target: use median bearing -> point camera towards the "middle" of depth-supported features
        beta_med = float(np.median(np.array(betas)))
        yaw_target = wrap_pi(yaw_cur + beta_med)

        # ---- smooth & rate limit ----
        if self.prev_yaw_cmd is None:
            yaw_cmd = yaw_target
        else:
            # low-pass on target
            yaw_lp = wrap_pi((1.0 - self.lp_alpha) * self.prev_yaw_cmd + self.lp_alpha * yaw_target)

            # rate limit
            dy = wrap_pi(yaw_lp - self.prev_yaw_cmd)
            dy_lim = max(-self.yaw_rate_max * dt, min(self.yaw_rate_max * dt, dy))
            yaw_cmd = wrap_pi(self.prev_yaw_cmd + dy_lim)

        yaw_dot_cmd = wrap_pi(yaw_cmd - (self.prev_yaw_cmd if self.prev_yaw_cmd is not None else yaw_cmd)) / dt
        self.prev_yaw_cmd = yaw_cmd

        self.publish_cmd(cmd_msg, yaw_cmd, yaw_dot_cmd)

    def publish_cmd(self, cmd_in: PositionCommand, yaw, yaw_dot):
        cmd_out = PositionCommand()
        cmd_out.header = cmd_in.header

        # copy everything
        cmd_out.position = cmd_in.position
        cmd_out.velocity = cmd_in.velocity
        cmd_out.acceleration = cmd_in.acceleration
        cmd_out.jerk = cmd_in.jerk
        cmd_out.yaw = yaw
        cmd_out.yaw_dot = yaw_dot

        # some PositionCommand have fields like kx/kv or trajectory_id, copy if exist
        for attr in ["kx", "kv", "trajectory_id", "trajectory_flag"]:
            if hasattr(cmd_in, attr):
                setattr(cmd_out, attr, getattr(cmd_in, attr))

        self.pub.publish(cmd_out)

def main():
    ApaceYawWrapper()
    rospy.spin()

if __name__ == "__main__":
    main()
