#!/usr/bin/env python3
import math
import numpy as np
import rospy

from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ApaceYawWrapperModes:
    def __init__(self):
        rospy.init_node("apace_yaw_wrapper_modes", anonymous=False)
        self.bridge = CvBridge()

        # ---- params ----
        self.cmd_in  = rospy.get_param("~cmd_in",  "/position_cmd_raw")
        self.cmd_out = rospy.get_param("~cmd_out", "/position_cmd")

        self.pose_topic    = rospy.get_param("~pose_topic",   "/mavros/local_position/pose")
        self.depth_topic   = rospy.get_param("~depth_topic",  "/iris_D435i/realsense/depth_camera/depth/image_raw")
        self.caminfo_topic = rospy.get_param("~caminfo_topic","/iris_D435i/realsense/depth_camera/color/camera_info")

        # 新增：跟踪点（障碍物中心）
        self.track_topic   = rospy.get_param("~track_topic", "/apace/track_point")

        # pass / lookahead / depth_roi / track_point / track_point_depth
        self.yaw_mode = rospy.get_param("~yaw_mode", "lookahead")

        # yaw dynamics
        self.yaw_rate_max = float(rospy.get_param("~yaw_rate_max", 0.8))
        self.yaw_kp       = float(rospy.get_param("~yaw_kp", 1.5))

        # depth_roi params（保持你原来的）
        self.safe_dist   = float(rospy.get_param("~safe_dist", 2.0))
        self.roi_w       = int(rospy.get_param("~roi_w", 120))
        self.roi_h       = int(rospy.get_param("~roi_h", 120))
        self.cand_deg    = float(rospy.get_param("~cand_deg", 90.0))
        self.step_deg    = float(rospy.get_param("~step_deg", 10.0))

        self.w_align = float(rospy.get_param("~w_align", 1.0))
        self.w_safe  = float(rospy.get_param("~w_safe", 2.0))

        # ---- state ----
        self.last_pose = None
        self.last_depth = None
        self.last_caminfo = None
        self.last_out_yaw = 0.0
        self.last_track_point = None

        # ---- pubs/subs ----
        self.pub = rospy.Publisher(self.cmd_out, PositionCommand, queue_size=10)

        rospy.Subscriber(self.cmd_in, PositionCommand, self.cb_cmd, queue_size=20)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=20)
        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)
        rospy.Subscriber(self.caminfo_topic, CameraInfo, self.cb_caminfo, queue_size=1)
        rospy.Subscriber(self.track_topic, Point, self.cb_track, queue_size=5)

        rospy.loginfo("APACE-Yaw Wrapper (modes) started.")
        rospy.loginfo(f"cmd_in={self.cmd_in} cmd_out={self.cmd_out} yaw_mode={self.yaw_mode}")
        rospy.loginfo(f"track_topic={self.track_topic}")

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_track(self, msg: Point):
        self.last_track_point = msg

    def cb_caminfo(self, msg: CameraInfo):
        self.last_caminfo = msg

    def cb_depth(self, msg: Image):
        try:
            if msg.encoding == "32FC1":
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1").astype(np.float32)
            elif msg.encoding == "16UC1":
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1").astype(np.uint16).astype(np.float32) * 0.001
            else:
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1").astype(np.float32)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Depth convert failed: {e}")
            return
        d = np.where(np.isfinite(d), d, 0.0)
        self.last_depth = d

    def get_yaw_from_pose(self):
        if self.last_pose is None:
            return None
        q = self.last_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def lookahead_yaw(self, cmd: PositionCommand):
        vx, vy = cmd.velocity.x, cmd.velocity.y
        v_norm = math.hypot(vx, vy)
        if v_norm > 0.15:
            return math.atan2(vy, vx)

        if self.last_pose is not None:
            px = self.last_pose.pose.position.x
            py = self.last_pose.pose.position.y
            dx = cmd.position.x - px
            dy = cmd.position.y - py
            if math.hypot(dx, dy) > 0.2:
                return math.atan2(dy, dx)

        return self.last_out_yaw

    def track_point_yaw(self):
        """让机头朝向 track_point（障碍物中心）"""
        if self.last_pose is None or self.last_track_point is None:
            return None
        px = self.last_pose.pose.position.x
        py = self.last_pose.pose.position.y
        ox = self.last_track_point.x
        oy = self.last_track_point.y
        dx, dy = ox - px, oy - py
        if math.hypot(dx, dy) < 0.2:
            return self.last_out_yaw
        return math.atan2(dy, dx)

    def roi_min_depth(self):
        if self.last_depth is None:
            return None
        d = self.last_depth
        h, w = d.shape[:2]
        cx, cy = w // 2, h // 2
        half_w = max(1, self.roi_w // 2)
        half_h = max(1, self.roi_h // 2)
        x0 = max(0, cx - half_w); x1 = min(w, cx + half_w)
        y0 = max(0, cy - half_h); y1 = min(h, cy + half_h)
        roi = d[y0:y1, x0:x1]
        valid = roi[(roi > 0.05) & (roi < 50.0)]
        if valid.size == 0:
            return None
        return float(np.percentile(valid, 5))

    def depth_roi_select_yaw(self, yaw_ref):
        """在 yaw_ref 周围找一个更安全的 yaw（简化版：用中心ROI距离做 proxy）"""
        dmin = self.roi_min_depth()
        if dmin is None:
            return yaw_ref

        cand_rad = math.radians(self.cand_deg)
        step_rad = math.radians(self.step_deg)

        # 这里 safe 分数用当前朝向的 ROI dmin 作为 proxy（你之前就是这个简化）
        safe = max(-1.0, min(1.0, (dmin - self.safe_dist) / max(1e-6, self.safe_dist)))

        best_yaw = yaw_ref
        best_score = -1e9
        for k in np.arange(-cand_rad, cand_rad + 1e-6, step_rad):
            yaw_cand = wrap_pi(yaw_ref + float(k))
            align = math.cos(wrap_pi(yaw_cand - yaw_ref))
            score = self.w_align * align + self.w_safe * safe
            score -= 0.05 * abs(wrap_pi(yaw_cand - self.last_out_yaw))
            if score > best_score:
                best_score = score
                best_yaw = yaw_cand
        return best_yaw

    def compute_yaw_and_rate(self, cmd_in: PositionCommand):
        # 1) yaw_ref
        if self.yaw_mode == "pass":
            yaw_des = cmd_in.yaw
        elif self.yaw_mode == "lookahead":
            yaw_des = self.lookahead_yaw(cmd_in)
        elif self.yaw_mode in ["track_point", "track_point_depth"]:
            tp = self.track_point_yaw()
            yaw_des = tp if tp is not None else self.lookahead_yaw(cmd_in)
        else:
            yaw_des = self.lookahead_yaw(cmd_in)

        # 2) depth 修正（可选）
        if self.yaw_mode in ["depth_roi", "track_point_depth"]:
            yaw_des = self.depth_roi_select_yaw(yaw_des)

        yaw_des = wrap_pi(yaw_des)

        # 3) yaw_dot：闭环
        yaw_cur = self.get_yaw_from_pose()
        if yaw_cur is not None:
            yaw_err = wrap_pi(yaw_des - yaw_cur)
            yaw_dot = max(-self.yaw_rate_max, min(self.yaw_rate_max, self.yaw_kp * yaw_err))
        else:
            yaw_err = wrap_pi(yaw_des - self.last_out_yaw)
            yaw_dot = max(-self.yaw_rate_max, min(self.yaw_rate_max, 2.0 * yaw_err))
        return yaw_des, yaw_dot

    def cb_cmd(self, cmd_in: PositionCommand):
        cmd_out = PositionCommand()
        cmd_out.header = cmd_in.header
        cmd_out.header.stamp = rospy.Time.now()

        cmd_out.position = cmd_in.position
        cmd_out.velocity = cmd_in.velocity
        cmd_out.acceleration = cmd_in.acceleration
        cmd_out.jerk = cmd_in.jerk

        yaw, yaw_dot = self.compute_yaw_and_rate(cmd_in)
        cmd_out.yaw = yaw
        cmd_out.yaw_dot = yaw_dot

        # 兼容字段透传
        for attr in ["kx", "kv", "trajectory_id", "trajectory_flag"]:
            if hasattr(cmd_in, attr):
                setattr(cmd_out, attr, getattr(cmd_in, attr))

        self.last_out_yaw = yaw
        self.pub.publish(cmd_out)

        rospy.loginfo_throttle(1.0, f"[{self.yaw_mode}] yaw_out={yaw:.3f}, yaw_dot_out={yaw_dot:.3f}")


if __name__ == "__main__":
    ApaceYawWrapperModes()
    rospy.spin()
