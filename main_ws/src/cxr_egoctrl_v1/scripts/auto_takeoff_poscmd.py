#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy

from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from quadrotor_msgs.msg import PositionCommand


def quat_to_yaw(q):
    """
    四元数转 yaw，单位 rad
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def smoothstep(s):
    """
    三次平滑插值 0~1
    """
    s = max(0.0, min(1.0, s))
    return s * s * (3.0 - 2.0 * s)


class AutoTakeoffPosCmd:
    def __init__(self):
        rospy.init_node("auto_takeoff_poscmd")

        # =========================
        # 参数
        # =========================
        self.target_z = rospy.get_param("~target_z", 1.0)
        self.takeoff_time = rospy.get_param("~takeoff_time", 6.0)
        self.auto_arm = rospy.get_param("~auto_arm", True)

        # 新增：起飞目标航向，默认朝正前方 yaw=0
        self.target_yaw = rospy.get_param("~target_yaw", 0.0)

        # 新增：到达目标高度后继续保持多久，再自动退出脚本
        self.final_hold_time = rospy.get_param("~final_hold_time", 3.0)

        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_cmd")
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)

        # =========================
        # 状态
        # =========================
        self.has_odom = False
        self.has_state = False

        self.current_odom = None
        self.current_state = State()

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = 0.0
        self.start_yaw = 0.0

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
            self.state_topic,
            State,
            self.state_cb,
            queue_size=10
        )

        self.arm_client = rospy.ServiceProxy(
            "/mavros/cmd/arming",
            CommandBool
        )

        rospy.loginfo("auto_takeoff_poscmd initialized.")
        rospy.loginfo("target_z          : %.2f", self.target_z)
        rospy.loginfo("takeoff_time      : %.2f", self.takeoff_time)
        rospy.loginfo("auto_arm          : %s", self.auto_arm)
        rospy.loginfo("target_yaw        : %.3f rad", self.target_yaw)
        rospy.loginfo("final_hold_time   : %.2f s", self.final_hold_time)
        rospy.loginfo("odom_topic        : %s", self.odom_topic)
        rospy.loginfo("cmd_topic         : %s", self.cmd_topic)

    # ============================================================
    # 回调
    # ============================================================
    def odom_cb(self, msg):
        self.current_odom = msg
        self.has_odom = True

    def state_cb(self, msg):
        self.current_state = msg
        self.has_state = True

    # ============================================================
    # PositionCommand
    # ============================================================
    def make_cmd(self, x, y, z, yaw):
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

        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.trajectory_id = 1

        return cmd

    def publish_cmd(self, x, y, z, yaw):
        cmd = self.make_cmd(x, y, z, yaw)
        self.cmd_pub.publish(cmd)

    # ============================================================
    # 等待初始化
    # ============================================================
    def wait_for_ready(self):
        rate = rospy.Rate(10)

        rospy.loginfo("Waiting for odom and MAVROS state...")

        while not rospy.is_shutdown():
            if self.has_odom and self.has_state and self.current_state.connected:
                break

            rospy.logwarn_throttle(
                1.0,
                "Waiting... odom=%s, state=%s, connected=%s",
                self.has_odom,
                self.has_state,
                self.current_state.connected if self.has_state else False
            )
            rate.sleep()

        p = self.current_odom.pose.pose.position
        q = self.current_odom.pose.pose.orientation

        self.start_x = p.x
        self.start_y = p.y
        self.start_z = p.z
        self.start_yaw = quat_to_yaw(q)

        rospy.loginfo(
            "Start pose locked: x=%.3f, y=%.3f, z=%.3f, current_yaw=%.3f, target_yaw=%.3f",
            self.start_x,
            self.start_y,
            self.start_z,
            self.start_yaw,
            self.target_yaw
        )

    # ============================================================
    # 等待 OFFBOARD
    # ============================================================
    def wait_for_offboard(self):
        rate = rospy.Rate(self.publish_rate)

        rospy.loginfo("Waiting for OFFBOARD mode...")

        while not rospy.is_shutdown() and self.current_state.mode != "OFFBOARD":
            self.publish_cmd(
                self.start_x,
                self.start_y,
                max(self.start_z, 0.10),
                self.target_yaw
            )

            rospy.loginfo_throttle(
                1.0,
                "Publishing hold. Waiting OFFBOARD... current mode=%s, armed=%s",
                self.current_state.mode,
                self.current_state.armed
            )

            rate.sleep()

        if rospy.is_shutdown():
            return False

        rospy.loginfo("OFFBOARD detected.")
        return True

    # ============================================================
    # 自动解锁
    # ============================================================
    def arm_if_needed(self):
        if not self.auto_arm:
            rospy.loginfo("auto_arm=False, skip arm.")
            return True

        if self.current_state.armed:
            rospy.loginfo("Vehicle already armed.")
            return True

        rospy.loginfo("Waiting for /mavros/cmd/arming service...")
        rospy.wait_for_service("/mavros/cmd/arming")

        rate = rospy.Rate(2)

        for i in range(10):
            if rospy.is_shutdown():
                return False

            if self.current_state.armed:
                rospy.loginfo("Vehicle armed.")
                return True

            try:
                resp = self.arm_client(True)
                rospy.loginfo(
                    "Arm request %d/10: success=%s, result=%s",
                    i + 1,
                    resp.success,
                    resp.result
                )
            except Exception as e:
                rospy.logwarn("Arm service call failed: %s", str(e))

            rate.sleep()

        if self.current_state.armed:
            rospy.loginfo("Vehicle armed.")
            return True

        rospy.logerr("Failed to arm vehicle.")
        return False

    # ============================================================
    # 起飞轨迹
    # ============================================================
    def execute_takeoff(self):
        rate = rospy.Rate(self.publish_rate)

        start_time = rospy.Time.now().to_sec()

        rospy.loginfo(
            "Start takeoff: z %.3f -> %.3f, duration=%.2f s, yaw=%.3f",
            self.start_z,
            self.target_z,
            self.takeoff_time,
            self.target_yaw
        )

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            t = now - start_time

            if t >= self.takeoff_time:
                break

            s = t / self.takeoff_time
            alpha = smoothstep(s)

            z_ref = self.start_z + (self.target_z - self.start_z) * alpha

            self.publish_cmd(
                self.start_x,
                self.start_y,
                z_ref,
                self.target_yaw
            )

            rospy.loginfo_throttle(
                1.0,
                "Taking off: x=%.3f, y=%.3f, z_ref=%.3f, target_z=%.3f, yaw=%.3f, mode=%s, armed=%s",
                self.start_x,
                self.start_y,
                z_ref,
                self.target_z,
                self.target_yaw,
                self.current_state.mode,
                self.current_state.armed
            )

            rate.sleep()

        rospy.loginfo("Takeoff trajectory finished.")

    # ============================================================
    # 起飞后短暂保持，然后自动退出
    # ============================================================
    def final_hold_then_exit(self):
        if self.final_hold_time <= 0.0:
            rospy.loginfo("final_hold_time <= 0, exit immediately.")
            return

        rate = rospy.Rate(self.publish_rate)
        hold_start = rospy.Time.now().to_sec()

        rospy.loginfo(
            "Final hold started: hold x=%.3f, y=%.3f, z=%.3f, yaw=%.3f for %.2f s",
            self.start_x,
            self.start_y,
            self.target_z,
            self.target_yaw,
            self.final_hold_time
        )

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            elapsed = now - hold_start

            if elapsed >= self.final_hold_time:
                break

            self.publish_cmd(
                self.start_x,
                self.start_y,
                self.target_z,
                self.target_yaw
            )

            rospy.loginfo_throttle(
                1.0,
                "Final holding: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f, mode=%s, armed=%s, remain=%.1f s",
                self.start_x,
                self.start_y,
                self.target_z,
                self.target_yaw,
                self.current_state.mode,
                self.current_state.armed,
                max(0.0, self.final_hold_time - elapsed)
            )

            rate.sleep()

        rospy.loginfo("Final hold finished. auto_takeoff_poscmd exits automatically.")

    # ============================================================
    # 主流程
    # ============================================================
    def run(self):
        self.wait_for_ready()

        ok = self.wait_for_offboard()
        if not ok:
            return

        ok = self.arm_if_needed()
        if not ok:
            return

        self.execute_takeoff()
        self.final_hold_then_exit()


if __name__ == "__main__":
    node = AutoTakeoffPosCmd()
    node.run()
