#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy

from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool


class AutoReturnLandPosCmd:
    def __init__(self):
        rospy.init_node("auto_return_land_poscmd")

        # =========================
        # 参数
        # =========================
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_cmd")

        # 返回目标点，默认回局部坐标原点
        self.target_x = rospy.get_param("~target_x", 0.0)
        self.target_y = rospy.get_param("~target_y", 0.0)

        # 定高返航高度
        # 默认 -999 表示使用运行脚本瞬间的当前高度
        # 如果想强制 1m 返航，运行时加 _return_z:=1.0
        self.return_z_param = rospy.get_param("~return_z", -999.0)

        # 降落目标高度
        # 实机建议 0.05，不建议直接设 0.0
        self.land_z = rospy.get_param("~land_z", 0.05)

        # 水平返航速度上限，单位 m/s
        self.return_speed = rospy.get_param("~return_speed", 0.25)

        # 水平返航最短/最长时间
        self.min_return_time = rospy.get_param("~min_return_time", 4.0)
        self.max_return_time = rospy.get_param("~max_return_time", 20.0)

        # 垂直下降时间，越大越慢
        self.descend_time = rospy.get_param("~descend_time", 12.0)

        # 到达 0,0 的判定阈值
        self.xy_tolerance = rospy.get_param("~xy_tolerance", 0.15)

        # 降落到低高度后，先保持一段时间
        self.ground_hold_time = rospy.get_param("~ground_hold_time", 4.0)

        # 自动上锁前，等待真实高度小于这个阈值
        # 你的上次日志里 current_z=0.167 时上锁失败，所以这里默认 0.12 更稳
        self.disarm_z_threshold = rospy.get_param("~disarm_z_threshold", 0.12)

        # 等待真实高度降低的最长时间
        self.disarm_wait_time = rospy.get_param("~disarm_wait_time", 8.0)

        # 如果等不到足够低，是否仍然尝试上锁
        # 实机建议 True：有时 VINS 的 z 有偏差，飞机已经落地但 z 还显示 0.15~0.20
        self.force_disarm_after_wait = rospy.get_param("~force_disarm_after_wait", True)

        # 是否自动上锁
        self.auto_disarm = rospy.get_param("~auto_disarm", True)

        # 发布频率
        self.rate_hz = rospy.get_param("~rate", 50.0)

        # 是否必须在 OFFBOARD 模式下运行
        self.require_offboard = rospy.get_param("~require_offboard", True)

        # =========================
        # 状态变量
        # =========================
        self.odom_received = False
        self.state_received = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.mav_state = State()

        # =========================
        # ROS 通信
        # =========================
        self.cmd_pub = rospy.Publisher(
            self.cmd_topic,
            PositionCommand,
            queue_size=10
        )

        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_cb,
            queue_size=10
        )

        self.state_sub = rospy.Subscriber(
            "/mavros/state",
            State,
            self.state_cb,
            queue_size=10
        )

        rospy.loginfo("Waiting for /mavros/cmd/arming service...")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        rospy.loginfo("auto_return_land_poscmd initialized.")
        rospy.loginfo("odom_topic: %s", self.odom_topic)
        rospy.loginfo("cmd_topic : %s", self.cmd_topic)
        rospy.loginfo("target   : x=%.2f, y=%.2f", self.target_x, self.target_y)
        rospy.loginfo("land_z   : %.2f", self.land_z)
        rospy.loginfo("descend_time: %.2f", self.descend_time)
        rospy.loginfo("disarm_z_threshold: %.2f", self.disarm_z_threshold)
        rospy.loginfo("auto_disarm: %s", self.auto_disarm)

    # =========================
    # 回调函数
    # =========================
    def state_cb(self, msg):
        self.mav_state = msg
        self.state_received = True

    def odom_cb(self, msg):
        self.odom_received = True

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        self.current_yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

    # =========================
    # 工具函数
    # =========================
    def quat_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def smooth_step(self, s):
        """
        五次平滑曲线：
        s 从 0 到 1，输出也从 0 到 1；
        起点和终点速度接近 0。
        """
        s = max(0.0, min(1.0, s))
        return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5

    def make_pos_cmd(self, x, y, z, yaw):
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"

        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z

        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0

        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0

        cmd.yaw = yaw
        cmd.yaw_dot = 0.0

        try:
            cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        except Exception:
            pass

        try:
            cmd.trajectory_id = 2
        except Exception:
            pass

        return cmd

    def publish_cmd(self, x, y, z, yaw):
        cmd = self.make_pos_cmd(x, y, z, yaw)
        self.cmd_pub.publish(cmd)

    def publish_hold(self, x, y, z, yaw, duration):
        rate = rospy.Rate(self.rate_hz)
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - start_time >= duration:
                break

            if not self.check_offboard_or_abort():
                return False

            self.publish_cmd(x, y, z, yaw)

            rospy.loginfo_throttle(
                1.0,
                "Holding: x=%.3f, y=%.3f, z=%.3f | current=(%.3f, %.3f, %.3f) mode=%s armed=%s",
                x,
                y,
                z,
                self.current_x,
                self.current_y,
                self.current_z,
                self.mav_state.mode,
                self.mav_state.armed
            )

            rate.sleep()

        return True

    # =========================
    # 等待 odom 和 MAVROS
    # =========================
    def wait_for_odom_and_state(self):
        rate = rospy.Rate(20)

        rospy.loginfo("Waiting for odom and MAVROS state...")

        while not rospy.is_shutdown():
            if self.odom_received and self.state_received and self.mav_state.connected:
                rospy.loginfo("Odom and MAVROS state ready.")
                rospy.loginfo(
                    "Current odom: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
                    self.current_x,
                    self.current_y,
                    self.current_z,
                    self.current_yaw
                )
                rospy.loginfo(
                    "Current MAVROS state: armed=%s, mode=%s, connected=%s",
                    self.mav_state.armed,
                    self.mav_state.mode,
                    self.mav_state.connected
                )
                return True

            rospy.logwarn_throttle(
                1.0,
                "Waiting... odom=%s, state=%s, connected=%s",
                self.odom_received,
                self.state_received,
                self.mav_state.connected
            )

            rate.sleep()

        return False

    # =========================
    # 等待 OFFBOARD
    # =========================
    def wait_for_offboard_with_hold(self, x, y, z, yaw):
        rospy.loginfo("Waiting for OFFBOARD while publishing hold command...")
        rospy.loginfo("Current mode: %s", self.mav_state.mode)

        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            self.publish_cmd(x, y, z, yaw)

            rospy.loginfo_throttle(
                1.0,
                "Waiting OFFBOARD... current mode=%s, armed=%s",
                self.mav_state.mode,
                self.mav_state.armed
            )

            if self.mav_state.mode == "OFFBOARD":
                rospy.loginfo("OFFBOARD detected.")
                return True

            rate.sleep()

        return False

    # =========================
    # 模式安全检查
    # =========================
    def check_offboard_or_abort(self):
        if not self.require_offboard:
            return True

        if self.mav_state.mode != "OFFBOARD":
            rospy.logerr(
                "Mode changed to %s, not OFFBOARD. Abort return/landing.",
                self.mav_state.mode
            )
            return False

        return True

    # =========================
    # 定高飞回 0,0
    # =========================
    def return_to_xy(self, start_x, start_y, start_z, target_x, target_y, yaw):
        dx = target_x - start_x
        dy = target_y - start_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.xy_tolerance:
            rospy.loginfo(
                "Already near target xy. dist=%.3f < tolerance=%.3f",
                dist,
                self.xy_tolerance
            )
            return True

        # 根据距离和速度自动计算返航时间
        if self.return_speed <= 0.01:
            return_time = self.min_return_time
        else:
            return_time = dist / self.return_speed

        return_time = max(self.min_return_time, min(return_time, self.max_return_time))

        rospy.loginfo(
            "Return to target xy: start=(%.3f, %.3f), target=(%.3f, %.3f), z=%.3f, dist=%.3f, return_time=%.2f s",
            start_x,
            start_y,
            target_x,
            target_y,
            start_z,
            dist,
            return_time
        )

        rate = rospy.Rate(self.rate_hz)
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if not self.check_offboard_or_abort():
                return False

            if not self.mav_state.armed:
                rospy.logerr("Vehicle disarmed during return. Abort.")
                return False

            now = rospy.Time.now().to_sec()
            t = now - start_time

            if t >= return_time:
                break

            s = t / return_time
            ss = self.smooth_step(s)

            x_ref = start_x + (target_x - start_x) * ss
            y_ref = start_y + (target_y - start_y) * ss
            z_ref = start_z

            self.publish_cmd(x_ref, y_ref, z_ref, yaw)

            current_dist = math.sqrt(
                (self.current_x - target_x) ** 2 +
                (self.current_y - target_y) ** 2
            )

            rospy.loginfo_throttle(
                0.5,
                "Returning... ref=(%.3f, %.3f, %.3f), current=(%.3f, %.3f, %.3f), dist_to_target=%.3f",
                x_ref,
                y_ref,
                z_ref,
                self.current_x,
                self.current_y,
                self.current_z,
                current_dist
            )

            rate.sleep()

        rospy.loginfo("Return trajectory command finished. Holding target xy before descent.")

        if not self.publish_hold(target_x, target_y, start_z, yaw, 2.0):
            return False

        current_dist = math.sqrt(
            (self.current_x - target_x) ** 2 +
            (self.current_y - target_y) ** 2
        )

        # 这里不要卡得太死，否则刚回到原点还在收敛时会误判失败
        if current_dist > max(self.xy_tolerance * 2.0, 0.35):
            rospy.logwarn(
                "Current xy is still far from target: dist=%.3f. Abort landing for safety.",
                current_dist
            )
            return False

        rospy.loginfo("Reached target xy. current_dist=%.3f", current_dist)
        return True

    # =========================
    # 垂直下降
    # =========================
    def descend_and_land(self, x, y, start_z, land_z, yaw):
        rospy.loginfo(
            "Start vertical descent at x=%.3f, y=%.3f: z %.3f -> %.3f, descend_time=%.2f s",
            x,
            y,
            start_z,
            land_z,
            self.descend_time
        )

        rate = rospy.Rate(self.rate_hz)
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if not self.check_offboard_or_abort():
                return False

            if not self.mav_state.armed:
                rospy.logerr("Vehicle disarmed during descent. Abort.")
                return False

            now = rospy.Time.now().to_sec()
            t = now - start_time

            if t >= self.descend_time:
                break

            s = t / self.descend_time
            ss = self.smooth_step(s)

            z_ref = start_z + (land_z - start_z) * ss

            self.publish_cmd(x, y, z_ref, yaw)

            rospy.loginfo_throttle(
                0.5,
                "Descending... ref_z=%.3f, current_z=%.3f, x=%.3f, y=%.3f",
                z_ref,
                self.current_z,
                self.current_x,
                self.current_y
            )

            rate.sleep()

        rospy.loginfo("Descent command finished. Holding near ground.")

        if not self.publish_hold(x, y, land_z, yaw, self.ground_hold_time):
            return False

        return True

    # =========================
    # 等待真实高度降低到可以上锁附近
    # =========================
    def wait_until_near_ground(self, x, y, z, yaw):
        rospy.loginfo(
            "Waiting until near ground before disarm: current_z <= %.3f, timeout=%.1f s",
            self.disarm_z_threshold,
            self.disarm_wait_time
        )

        rate = rospy.Rate(self.rate_hz)
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if not self.check_offboard_or_abort():
                return False

            # 如果已经上锁，直接返回
            if not self.mav_state.armed:
                rospy.loginfo("Vehicle already disarmed while waiting near ground.")
                return True

            now = rospy.Time.now().to_sec()

            # 继续发布低高度目标，帮助飞机真正贴近地面
            self.publish_cmd(x, y, z, yaw)

            rospy.loginfo_throttle(
                0.5,
                "Near-ground waiting... current_z=%.3f, target_z=%.3f, threshold=%.3f, mode=%s, armed=%s",
                self.current_z,
                z,
                self.disarm_z_threshold,
                self.mav_state.mode,
                self.mav_state.armed
            )

            if self.current_z <= self.disarm_z_threshold:
                rospy.loginfo(
                    "Near ground detected. current_z=%.3f <= %.3f",
                    self.current_z,
                    self.disarm_z_threshold
                )
                return True

            if now - start_time > self.disarm_wait_time:
                rospy.logwarn(
                    "Timeout waiting near ground. current_z=%.3f, threshold=%.3f",
                    self.current_z,
                    self.disarm_z_threshold
                )

                if self.force_disarm_after_wait:
                    rospy.logwarn(
                        "force_disarm_after_wait=True, will try to disarm anyway."
                    )
                    return True
                else:
                    rospy.logwarn(
                        "force_disarm_after_wait=False, skip auto disarm. Please disarm manually."
                    )
                    return False

            rate.sleep()

        return False

    # =========================
    # 自动上锁
    # =========================
    def disarm(self):
        if not self.auto_disarm:
            rospy.logwarn("auto_disarm disabled. Please disarm manually.")
            return True

        rospy.loginfo("Disarming vehicle...")

        for i in range(12):
            if not self.mav_state.armed:
                rospy.loginfo("Vehicle already disarmed.")
                return True

            try:
                resp = self.arming_client(False)

                rospy.logwarn(
                    "Disarm response: success=%s, result=%s",
                    resp.success,
                    resp.result
                )

                if resp.success:
                    rospy.loginfo("Vehicle disarmed.")
                    return True
                else:
                    rospy.logwarn("Disarm failed, retrying...")

            except rospy.ServiceException as e:
                rospy.logerr("Disarm service error: %s", str(e))

            # 继续给低高度 setpoint，避免 OFFBOARD setpoint 中断
            self.publish_cmd(self.target_x, self.target_y, self.land_z, self.current_yaw)

            rospy.sleep(0.5)

        rospy.logerr("Failed to disarm vehicle. Please disarm manually with RC.")
        return False

    # =========================
    # 主流程
    # =========================
    def run(self):
        # 1. 等待 odom 和 MAVROS 状态
        if not self.wait_for_odom_and_state():
            return

        # 2. 锁定当前状态
        start_x = self.current_x
        start_y = self.current_y
        start_z = self.current_z
        start_yaw = self.current_yaw

        # 如果用户没有指定 return_z，就用当前高度定高返航
        if self.return_z_param < -100.0:
            return_z = start_z
        else:
            return_z = self.return_z_param

        rospy.loginfo("Return-land reference locked:")
        rospy.loginfo(
            "start_x=%.3f, start_y=%.3f, start_z=%.3f, return_z=%.3f, yaw=%.3f",
            start_x,
            start_y,
            start_z,
            return_z,
            start_yaw
        )

        # 3. 如果还没在 OFFBOARD，就等待 OFFBOARD
        # 等待过程中持续发布当前点悬停，避免切 OFFBOARD 时没有 setpoint
        if self.require_offboard:
            if self.mav_state.mode != "OFFBOARD":
                rospy.logwarn(
                    "Current mode is %s, not OFFBOARD. Waiting for OFFBOARD...",
                    self.mav_state.mode
                )
                if not self.wait_for_offboard_with_hold(start_x, start_y, start_z, start_yaw):
                    return

        # 4. 检查是否 armed
        if not self.mav_state.armed:
            rospy.logerr("Vehicle is not armed. Abort return landing.")
            return

        # 5. 先短暂悬停
        rospy.loginfo("Holding before return...")
        if not self.publish_hold(start_x, start_y, return_z, start_yaw, 1.0):
            return

        # 6. 定高飞回 0,0
        if not self.return_to_xy(
            start_x,
            start_y,
            return_z,
            self.target_x,
            self.target_y,
            start_yaw
        ):
            rospy.logerr("Return to xy failed. Abort landing.")
            return

        # 7. 到达 0,0 后垂直下降
        if not self.descend_and_land(
            self.target_x,
            self.target_y,
            return_z,
            self.land_z,
            start_yaw
        ):
            rospy.logerr("Descent failed.")
            return

        # 8. 下降完成后，继续等真实高度足够低
        near_ground_ok = self.wait_until_near_ground(
            self.target_x,
            self.target_y,
            self.land_z,
            start_yaw
        )

        if not near_ground_ok:
            rospy.logwarn(
                "Near-ground condition not satisfied. Skip auto disarm. Please disarm manually."
            )
            return

        # 9. 自动上锁
        self.disarm()

        rospy.loginfo("Auto return and landing finished.")


if __name__ == "__main__":
    try:
        node = AutoReturnLandPosCmd()
        node.run()
    except rospy.ROSInterruptException:
        pass
