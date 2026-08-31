#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy

from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def set_kx_kv_if_exist(msg: PositionCommand, kx=(6.0, 6.0, 6.0), kv=(3.5, 3.5, 3.5)):
    """
    兼容不同 PositionCommand 定义：
    - kx/kv 可能是 geometry_msgs/Vector3
    - 或 float64[3]
    """
    if hasattr(msg, "kx"):
        try:
            msg.kx.x, msg.kx.y, msg.kx.z = kx
        except Exception:
            try:
                msg.kx = list(kx)
            except Exception:
                pass

    if hasattr(msg, "kv"):
        try:
            msg.kv.x, msg.kv.y, msg.kv.z = kv
        except Exception:
            try:
                msg.kv = list(kv)
            except Exception:
                pass


class GoalOrbitObstaclePub:
    """
    GO: 直奔目标 (gx,gy,gz) —— 以 position setpoint 形式发布
    触发：中心 ROI 深度 < trigger_dist
    ORBIT：围绕“障碍中心”走半圈（累计角度>=orbit_angle_need）或前方持续安全 -> 退出

    输出：
      - /position_cmd_raw (PositionCommand)：位置轨迹（yaw 不写死）
      - /apace/track_point (Point)：障碍中心点（给 yaw wrapper 做 “机头对准障碍物”）
    """

    def __init__(self):
        rospy.init_node("pub_goal_orbit_obstacle", anonymous=False)
        self.bridge = CvBridge()

        # ---- params ----
        self.topic = rospy.get_param("~topic", "/position_cmd_raw")
        self.rate_hz = float(rospy.get_param("~rate", 30.0))

        self.gx = float(rospy.get_param("~gx", 6.0))
        self.gy = float(rospy.get_param("~gy", 0.0))
        self.gz = float(rospy.get_param("~gz", 1.0))

        self.pose_topic = rospy.get_param("~pose_topic", "/mavros/local_position/pose")
        self.depth_topic = rospy.get_param("~depth_topic", "/iris_D435i/realsense/depth_camera/depth/image_raw")

        # ✅ 关键：给 yaw wrapper 的“跟踪点”
        self.track_topic = rospy.get_param("~track_topic", "/apace/track_point")

        self.trigger_dist = float(rospy.get_param("~trigger_dist", 1.8))
        self.safe_dist = float(rospy.get_param("~safe_dist", 2.3))
        self.safe_hold_frames = int(rospy.get_param("~safe_hold_frames", 20))

        # ORBIT 参数
        self.orbit_omega = float(rospy.get_param("~orbit_omega", 0.35))  # rad/s
        self.orbit_margin = float(rospy.get_param("~orbit_margin", 0.8))
        self.orbit_r_min = float(rospy.get_param("~orbit_r_min", 2.0))
        self.orbit_r_max = float(rospy.get_param("~orbit_r_max", 4.0))
        self.orbit_angle_need = float(rospy.get_param("~orbit_angle_need", math.pi))

        # ✅ 防撞关键：障碍物膨胀半径（至少要大于：障碍物外接半径 + 机体半径 + 安全裕度）
        # 你现在的“墙/box”建议先从 2.2~2.8 调起，宁可大一点先不撞
        self.obstacle_inflate = float(rospy.get_param("~obstacle_inflate", 2.5))

        # ROI
        self.roi_w = int(rospy.get_param("~roi_w", 120))
        self.roi_h = int(rospy.get_param("~roi_h", 120))

        # GO 阶段使用的参考速度（给 lookahead yaw 参考用，不是必须）
        self.v_xy = float(rospy.get_param("~v_xy", 1.0))

        # PositionCommand 控制增益（如果你的控制器依赖）
        self.kx = (
            float(rospy.get_param("~kx_x", 6.0)),
            float(rospy.get_param("~kx_y", 6.0)),
            float(rospy.get_param("~kx_z", 6.0)),
        )
        self.kv = (
            float(rospy.get_param("~kv_x", 3.5)),
            float(rospy.get_param("~kv_y", 3.5)),
            float(rospy.get_param("~kv_z", 3.5)),
        )

        # ---- state ----
        self.last_pose = None
        self.last_depth = None

        self.mode = "GO"  # GO / ORBIT
        self.orbit_center = None  # (ox, oy)
        self.orbit_radius = None
        self.orbit_sign = +1

        # ORBIT 相位推进（不依赖当前位置微小变化）
        self.theta_ref = None
        self.theta_acc = 0.0

        self.safe_cnt = 0
        self.traj_id = 0

        # ---- ros IO ----
        self.pub = rospy.Publisher(self.topic, PositionCommand, queue_size=10)
        self.track_pub = rospy.Publisher(self.track_topic, Point, queue_size=10)

        rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=20)
        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)

        rospy.loginfo("pub_goal_orbit_obstacle started.")
        rospy.loginfo("GOAL=(%.2f, %.2f, %.2f), trigger_dist=%.2f, orbit_need=%.2f rad",
                      self.gx, self.gy, self.gz, self.trigger_dist, self.orbit_angle_need)
        rospy.loginfo("orbit_r_min=%.2f orbit_r_max=%.2f orbit_margin=%.2f obstacle_inflate=%.2f",
                      self.orbit_r_min, self.orbit_r_max, self.orbit_margin, self.obstacle_inflate)
        rospy.loginfo("track_topic=%s (yaw wrapper should use yaw_mode:=track_point or track_point_depth)",
                      self.track_topic)

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_depth(self, msg: Image):
        try:
            if msg.encoding == "32FC1":
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1").astype(np.float32)
            elif msg.encoding == "16UC1":
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1").astype(np.uint16).astype(np.float32) * 0.001
            else:
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1").astype(np.float32)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "depth convert failed: %s", str(e))
            return

        d = np.where(np.isfinite(d), d, 0.0)
        self.last_depth = d

    def roi_depth_percentile(self, p=5):
        if self.last_depth is None:
            return None
        d = self.last_depth
        h, w = d.shape[:2]
        cx, cy = w // 2, h // 2
        hw = max(1, self.roi_w // 2)
        hh = max(1, self.roi_h // 2)
        x0, x1 = max(0, cx - hw), min(w, cx + hw)
        y0, y1 = max(0, cy - hh), min(h, cy + hh)

        roi = d[y0:y1, x0:x1]
        valid = roi[(roi > 0.10) & (roi < 50.0)]
        if valid.size == 0:
            return None
        return float(np.percentile(valid, p))

    def choose_orbit_direction(self, px, py, ox, oy):
        """根据从当前位置绕到 goal 的相对几何，选顺/逆时针，使绕行更容易走向 goal"""
        gx, gy = self.gx, self.gy
        ax, ay = px - ox, py - oy
        bx, by = gx - ox, gy - oy
        cross = ax * by - ay * bx
        return +1 if cross > 0 else -1

    def enter_orbit(self, d_front):
        """
        用当前 yaw + 前方深度，粗略估计一个“障碍中心点”在前方。
        （对墙来说中心不真实，但足够触发出“绕行 + 机头对准 center”的行为）
        """
        p = self.last_pose.pose.position
        q = self.last_pose.pose.orientation
        yaw = quat_to_yaw(q)

        # ---- 计算绕行半径 r ----
        # 先用前方深度给一个参考 r0，再加 margin，再 clamp
        r0 = (d_front + self.orbit_margin) if (d_front is not None) else (self.orbit_r_min)
        r0 = max(self.orbit_r_min, min(self.orbit_r_max, r0))

        # ✅ 最关键：强制最小“膨胀半径”，避免太贴障碍
        r = max(r0, self.obstacle_inflate)

        # ---- 估计 center（在当前朝向前方 r 处）----
        ox = p.x + r * math.cos(yaw)
        oy = p.y + r * math.sin(yaw)

        self.orbit_center = (ox, oy)
        self.orbit_radius = r
        self.orbit_sign = self.choose_orbit_direction(p.x, p.y, ox, oy)

        # 相位初始化：用当前位置相对 center 的角度
        theta = math.atan2(p.y - oy, p.x - ox)
        self.theta_ref = theta
        self.theta_acc = 0.0
        self.safe_cnt = 0

        self.mode = "ORBIT"
        rospy.logwarn("ENTER ORBIT: center=(%.2f,%.2f) r=%.2f sign=%+d (d_front=%.2f)",
                      ox, oy, r, self.orbit_sign, -1.0 if d_front is None else d_front)

    def exit_orbit_if_ready(self, d_front):
        # A: 半圈完成
        if self.theta_acc >= self.orbit_angle_need:
            rospy.logwarn("EXIT ORBIT: angle_acc=%.2f rad reached", self.theta_acc)
            return True

        # B: 前方持续安全
        if d_front is not None and d_front > self.safe_dist:
            self.safe_cnt += 1
        else:
            self.safe_cnt = 0

        if self.safe_cnt >= self.safe_hold_frames:
            rospy.logwarn("EXIT ORBIT: front safe for %d frames", self.safe_cnt)
            return True

        return False

    def go_step(self):
        p = self.last_pose.pose.position
        dx = self.gx - p.x
        dy = self.gy - p.y
        dist = math.hypot(dx, dy)
        if dist < 0.25:
            return self.gx, self.gy, 0.0, 0.0

        ux, uy = dx / dist, dy / dist
        vx, vy = self.v_xy * ux, self.v_xy * uy
        return self.gx, self.gy, vx, vy

    def orbit_step(self, dt):
        ox, oy = self.orbit_center
        r = self.orbit_radius

        dtheta = self.orbit_omega * dt
        self.theta_ref = wrap_pi(self.theta_ref + self.orbit_sign * dtheta)
        self.theta_acc += abs(dtheta)

        x_des = ox + r * math.cos(self.theta_ref)
        y_des = oy + r * math.sin(self.theta_ref)

        # 切向速度（给 lookahead 参考；但机头朝向将由 track_point 决定）
        vx = -self.orbit_sign * r * self.orbit_omega * math.sin(self.theta_ref)
        vy =  self.orbit_sign * r * self.orbit_omega * math.cos(self.theta_ref)

        return x_des, y_des, vx, vy

    def publish_track_point(self):
        """持续发布障碍中心点，让 yaw wrapper 机头对准它"""
        if self.orbit_center is None:
            return
        ox, oy = self.orbit_center
        pt = Point()
        pt.x = ox
        pt.y = oy
        pt.z = self.gz
        self.track_pub.publish(pt)

    def publish_cmd(self, x, y, z, vx, vy):
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = 0.0

        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0

        msg.jerk.x = 0.0
        msg.jerk.y = 0.0
        msg.jerk.z = 0.0

        # yaw 留给 wrapper（不要写死）
        msg.yaw = 0.0
        msg.yaw_dot = 0.0

        # 常见必须字段（避免控制器忽略）
        set_kx_kv_if_exist(msg, self.kx, self.kv)
        if hasattr(msg, "trajectory_id"):
            msg.trajectory_id = self.traj_id
        if hasattr(msg, "trajectory_flag"):
            msg.trajectory_flag = 1
        self.traj_id += 1

        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last_t = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.last_pose is None:
                rate.sleep()
                continue

            now_t = rospy.Time.now().to_sec()
            dt = max(1.0 / self.rate_hz, now_t - last_t)
            last_t = now_t

            d_front = self.roi_depth_percentile(p=5)

            if self.mode == "GO":
                if d_front is not None and d_front < self.trigger_dist:
                    self.enter_orbit(d_front)

            if self.mode == "ORBIT":
                # ✅ 一直发布 track_point（机头对准障碍中心）
                self.publish_track_point()

                if self.exit_orbit_if_ready(d_front):
                    self.mode = "GO"
                else:
                    x, y, vx, vy = self.orbit_step(dt)
                    self.publish_cmd(x, y, self.gz, vx, vy)
                    rospy.loginfo_throttle(
                        1.0,
                        "[ORBIT] d=%.2f angle=%.2f safe_cnt=%d r=%.2f xdes=%.2f ydes=%.2f",
                        -1.0 if d_front is None else d_front,
                        self.theta_acc, self.safe_cnt,
                        -1.0 if self.orbit_radius is None else self.orbit_radius,
                        x, y
                    )
                    rate.sleep()
                    continue

            # GO
            x, y, vx, vy = self.go_step()
            self.publish_cmd(x, y, self.gz, vx, vy)
            rospy.loginfo_throttle(
                1.0,
                "[GO] d=%.2f xdes=%.2f ydes=%.2f",
                -1.0 if d_front is None else d_front, x, y
            )
            rate.sleep()


if __name__ == "__main__":
    node = GoalOrbitObstaclePub()
    node.run()
