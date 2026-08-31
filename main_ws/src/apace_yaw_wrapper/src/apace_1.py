#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy

from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# =========================
# 参数集中区：你只改这里就行
# =========================

GOAL_X = 10.0   
GOAL_Y = 0.0
GOAL_Z = 1.0

PUB_TOPIC = "/position_cmd"      # 直接发到控制器输入（你验证过这样能旋转）
POSE_TOPIC = "/mavros/local_position/pose"
DEPTH_TOPIC = "/iris_D435i/realsense/depth_camera/depth/image_raw"

RATE_HZ = 30.0

# 触发绕障：前方ROI深度 < TRIGGER_DIST
TRIGGER_DIST = 1.5       # m
SAFE_DIST = 2.0          # m (用于“安全判定”)
SAFE_HOLD_FRAMES = 25    # 连续安全多少帧认为“前方安全”

# 绕障轨道
ORBIT_OMEGA = 0.35       # rad/s
ORBIT_ANGLE_NEED = math.pi  # 半圈：pi；想飞半圈就保持 pi

ORBIT_MARGIN = 1.5       # 在测得前方深度基础上，额外加的半径裕量
ORBIT_R_MIN = 1.6       # ✅ 强制最小半径（太小必撞）
ORBIT_R_MAX = 5.0

# 绕障期间机头始终朝向障碍物中心：yaw = atan2(oy - y, ox - x)

# ESCAPE：绕完半圈后，沿切线方向“脱离”一小段，再去 GO（防止立刻二次触发）
ESCAPE_TIME = 2.0        # s
ESCAPE_SPEED = 0.8       # m/s

# 冷却防抖：退出 ORBIT 后，多少秒内禁止再次触发
RETRIGGER_COOLDOWN_S = 2.5

# ROI取深度百分位（越小越保守）
ROI_W = 120
ROI_H = 120
ROI_PERCENTILE = 5

# 直行速度（GO模式速度参考）
GO_VXY = 0.8

# 如果你的 PositionCommand 里需要 kx/kv（你的 cxr_egoctrl_v1 很可能需要）
KX = (6.0, 6.0, 6.0)
KV = (3.5, 3.5, 3.5)


# =========================
# 工具函数
# =========================

def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def set_kx_kv_if_exist(msg: PositionCommand, kx=KX, kv=KV):
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

def clamp(x, a, b):
    return max(a, min(b, x))


# =========================
# 主节点
# =========================

class GoalOrbitOneFile:
    """
    状态机：
      GO      : 朝目标直飞
      ORBIT   : 围绕“障碍中心”转半圈，机头朝向障碍
      ESCAPE  : 绕完后沿切线方向离开一小段，避免立即二次触发
    """

    def __init__(self):
        rospy.init_node("apace_goal_orbit_onefile", anonymous=False)
        self.bridge = CvBridge()

        # state
        self.last_pose = None
        self.last_depth = None

        self.mode = "GO"  # GO / ORBIT / ESCAPE

        self.orbit_center = None   # (ox, oy)
        self.orbit_radius = None   # r
        self.orbit_sign = +1       # +1 CCW / -1 CW

        self.theta_ref = None
        self.theta_acc = 0.0

        self.safe_cnt = 0
        self.cooldown_until = 0.0

        # escape
        self.escape_end_t = 0.0
        self.escape_vx = 0.0
        self.escape_vy = 0.0

        self.pub = rospy.Publisher(PUB_TOPIC, PositionCommand, queue_size=10)
        rospy.Subscriber(POSE_TOPIC, PoseStamped, self.cb_pose, queue_size=10)
        rospy.Subscriber(DEPTH_TOPIC, Image, self.cb_depth, queue_size=1)

        rospy.loginfo("apace_goal_orbit_onefile started.")
        rospy.loginfo("GOAL=(%.2f, %.2f, %.2f) topic=%s", GOAL_X, GOAL_Y, GOAL_Z, PUB_TOPIC)

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

    def roi_depth_percentile(self, p=ROI_PERCENTILE):
        if self.last_depth is None:
            return None
        d = self.last_depth
        h, w = d.shape[:2]
        cx, cy = w // 2, h // 2
        hw = max(1, ROI_W // 2)
        hh = max(1, ROI_H // 2)

        x0, x1 = max(0, cx - hw), min(w, cx + hw)
        y0, y1 = max(0, cy - hh), min(h, cy + hh)

        roi = d[y0:y1, x0:x1]
        valid = roi[(roi > 0.05) & (roi < 50.0)]
        if valid.size == 0:
            return None
        return float(np.percentile(valid, p))

    def estimate_obstacle_center(self, yaw_cur, d_front):
        """
        用“前方深度” + 当前位置 + 当前yaw 估计障碍中心点。
        这是你当前仿真验证最容易成功的近似。
        """
        p = self.last_pose.pose.position
        # r = d_front + margin，然后 clamp 到 [r_min, r_max]
        r = clamp(d_front + ORBIT_MARGIN, ORBIT_R_MIN, ORBIT_R_MAX)
        ox = p.x + r * math.cos(yaw_cur)
        oy = p.y + r * math.sin(yaw_cur)
        return ox, oy, r

    def choose_orbit_direction(self, px, py, ox, oy):
        """
        根据：从障碍指向当前点向量 a，与障碍指向目标向量 b 的叉积，
        选择绕行方向，倾向于让绕行后更容易接近目标。
        """
        ax, ay = px - ox, py - oy
        bx, by = GOAL_X - ox, GOAL_Y - oy
        cross = ax * by - ay * bx
        return +1 if cross > 0 else -1

    def enter_orbit(self, d_front):
        p = self.last_pose.pose.position
        q = self.last_pose.pose.orientation
        yaw_cur = quat_to_yaw(q)

        ox, oy, r = self.estimate_obstacle_center(yaw_cur, d_front)
        sign = self.choose_orbit_direction(p.x, p.y, ox, oy)

        self.orbit_center = (ox, oy)
        self.orbit_radius = r
        self.orbit_sign = sign

        self.theta_ref = math.atan2(p.y - oy, p.x - ox)
        self.theta_acc = 0.0
        self.safe_cnt = 0

        self.mode = "ORBIT"
        rospy.logwarn("ENTER ORBIT: center=(%.2f, %.2f) r=%.2f sign=%+d", ox, oy, r, sign)

    def should_exit_orbit(self, d_front):
        # A) 半圈完成
        if self.theta_acc >= ORBIT_ANGLE_NEED:
            rospy.logwarn("EXIT ORBIT: angle_acc=%.2f reached", self.theta_acc)
            return True

        # B) 前方持续安全
        if d_front is not None and d_front > SAFE_DIST:
            self.safe_cnt += 1
        else:
            self.safe_cnt = 0

        if self.safe_cnt >= SAFE_HOLD_FRAMES:
            rospy.logwarn("EXIT ORBIT: front safe %d frames", self.safe_cnt)
            return True

        return False

    def start_escape(self, dt):
        """
        ESCAPE：沿切线方向离开障碍一小段，避免刚退出又看到墙马上二次触发。
        切线方向由 orbit_sign 和 theta_ref 决定。
        """
        # 切向单位向量（绕行方向）
        # 对于 CCW：t = (-sin, cos)
        # 对于 CW ：t = ( sin,-cos) 等价于 sign 处理
        t_x = -self.orbit_sign * math.sin(self.theta_ref)
        t_y =  self.orbit_sign * math.cos(self.theta_ref)

        self.escape_vx = ESCAPE_SPEED * t_x
        self.escape_vy = ESCAPE_SPEED * t_y

        now = rospy.Time.now().to_sec()
        self.escape_end_t = now + ESCAPE_TIME
        self.mode = "ESCAPE"

        # 启动冷却（ESCAPE + 初期 GO 都禁止再触发）
        self.cooldown_until = now + RETRIGGER_COOLDOWN_S

        rospy.logwarn("ORBIT->ESCAPE: vx=%.2f vy=%.2f escape_time=%.2fs cooldown=%.2fs",
                      self.escape_vx, self.escape_vy, ESCAPE_TIME, RETRIGGER_COOLDOWN_S)

    def go_command(self):
        p = self.last_pose.pose.position
        dx = GOAL_X - p.x
        dy = GOAL_Y - p.y
        dist = math.hypot(dx, dy)

        # 到达目标附近：停住
        if dist < 0.3:
            return GOAL_X, GOAL_Y, 0.0, 0.0

        ux, uy = dx / dist, dy / dist
        vx, vy = GO_VXY * ux, GO_VXY * uy
        return GOAL_X, GOAL_Y, vx, vy

    def orbit_command(self, dt):
        ox, oy = self.orbit_center
        r = self.orbit_radius

        dtheta = ORBIT_OMEGA * dt
        self.theta_ref = wrap_pi(self.theta_ref + self.orbit_sign * dtheta)
        self.theta_acc += abs(dtheta)

        x = ox + r * math.cos(self.theta_ref)
        y = oy + r * math.sin(self.theta_ref)

        # 切向速度（轨迹平滑 + 控制器参考）
        vx = -self.orbit_sign * r * ORBIT_OMEGA * math.sin(self.theta_ref)
        vy =  self.orbit_sign * r * ORBIT_OMEGA * math.cos(self.theta_ref)

        return x, y, vx, vy

    def escape_command(self):
        """
        ESCAPE 时位置用“当前位置 + 小速度”，让控制器自然跟随
        """
        p = self.last_pose.pose.position
        x = p.x + self.escape_vx * (1.0 / RATE_HZ)
        y = p.y + self.escape_vy * (1.0 / RATE_HZ)
        return x, y, self.escape_vx, self.escape_vy

    def publish_cmd(self, x, y, z, vx, vy, yaw, yaw_dot):
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

        # ✅ yaw 显式控制：绕障期间机头看向障碍物；GO/ESCAPE 时机头朝向速度方向（前进）
        msg.yaw = yaw
        msg.yaw_dot = yaw_dot

        set_kx_kv_if_exist(msg, KX, KV)

        # 一些版本可能需要这些
        if hasattr(msg, "trajectory_flag"):
            msg.trajectory_flag = 1
        if hasattr(msg, "trajectory_id"):
            msg.trajectory_id = int(msg.header.stamp.to_nsec() % 1000000000)

        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(RATE_HZ)
        last_t = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.last_pose is None:
                rate.sleep()
                continue

            now_t = rospy.Time.now().to_sec()
            dt = max(1.0 / RATE_HZ, now_t - last_t)
            last_t = now_t

            d_front = self.roi_depth_percentile(p=ROI_PERCENTILE)

            p = self.last_pose.pose.position
            q = self.last_pose.pose.orientation
            yaw_cur = quat_to_yaw(q)

            # =====================
            # 状态机
            # =====================

            if self.mode == "GO":
                # 冷却期内禁止触发
                if now_t >= self.cooldown_until:
                    if d_front is not None and d_front < TRIGGER_DIST:
                        self.enter_orbit(d_front)

                # GO command
                x, y, vx, vy = self.go_command()

                # GO 时机头朝速度方向（保持“朝前”）
                yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.05 else yaw_cur
                yaw_dot = 0.0

                self.publish_cmd(x, y, GOAL_Z, vx, vy, yaw, yaw_dot)

                rospy.loginfo_throttle(1.0, "[GO] d=%.2f pos=(%.2f,%.2f) goal=(%.1f,%.1f)",
                                       -1.0 if d_front is None else d_front, p.x, p.y, GOAL_X, GOAL_Y)

            elif self.mode == "ORBIT":
                # ORBIT command
                x, y, vx, vy = self.orbit_command(dt)

                ox, oy = self.orbit_center

                # ✅ ORBIT：机头始终看向障碍中心
                yaw = math.atan2(oy - y, ox - x)
                yaw_dot = 0.0

                self.publish_cmd(x, y, GOAL_Z, vx, vy, yaw, yaw_dot)

                rospy.loginfo_throttle(1.0, "[ORBIT] d=%.2f angle=%.2f r=%.2f",
                                       -1.0 if d_front is None else d_front, self.theta_acc, self.orbit_radius)

                # exit?
                if self.should_exit_orbit(d_front):
                    self.start_escape(dt)

            elif self.mode == "ESCAPE":
                # ESCAPE command
                x, y, vx, vy = self.escape_command()

                # ESCAPE：机头朝速度方向（也算“朝前”）
                yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.05 else yaw_cur
                yaw_dot = 0.0

                self.publish_cmd(x, y, GOAL_Z, vx, vy, yaw, yaw_dot)

                if now_t >= self.escape_end_t:
                    rospy.logwarn("ESCAPE->GO")
                    self.mode = "GO"

            rate.sleep()


if __name__ == "__main__":
    node = GoalOrbitOneFile()
    node.run()
