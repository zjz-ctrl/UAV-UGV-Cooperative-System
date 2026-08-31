#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy

from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from quadrotor_msgs.msg import PositionCommand


class AutoTakeoffGCS:
    def __init__(self):
        rospy.init_node("auto_takeoff_gcs")

        self.target_z = rospy.get_param("~target_z", 1.0)
        self.takeoff_time = rospy.get_param("~takeoff_time", 6.0)
        self.auto_arm = rospy.get_param("~auto_arm", True)
        self.auto_offboard = rospy.get_param("~auto_offboard", True)
        self.rate_hz = rospy.get_param("~rate", 30.0)

        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_cmd")

        self.state = State()
        self.odom = None

        self.has_odom = False
        self.has_state = False

        self.pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=10)

        rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)

        rospy.loginfo("auto_takeoff_gcs started")
        rospy.loginfo(
            "target_z=%.2f, takeoff_time=%.2f, auto_arm=%s, auto_offboard=%s",
            self.target_z,
            self.takeoff_time,
            str(self.auto_arm),
            str(self.auto_offboard)
        )
        rospy.loginfo("odom_topic=%s, cmd_topic=%s", self.odom_topic, self.cmd_topic)

    def state_cb(self, msg):
        self.state = msg
        self.has_state = True

    def odom_cb(self, msg):
        self.odom = msg
        self.has_odom = True

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_cmd(self, x, y, z, yaw):
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

        self.pub.publish(cmd)

    def call_offboard(self):
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=2.0)
            set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            resp = set_mode(custom_mode="OFFBOARD")
            rospy.loginfo("Request OFFBOARD: mode_sent=%s", str(resp.mode_sent))
            return resp.mode_sent
        except Exception as e:
            rospy.logwarn("Request OFFBOARD failed: %s", str(e))
            return False

    def call_arm(self):
        try:
            rospy.wait_for_service("/mavros/cmd/arming", timeout=2.0)
            arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            resp = arm_srv(True)
            rospy.loginfo("Request arm: success=%s", str(resp.success))
            return resp.success
        except Exception as e:
            rospy.logwarn("Request arm failed: %s", str(e))
            return False

    def run(self):
        rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("Waiting for odom and mavros state...")
        while not rospy.is_shutdown() and (not self.has_odom or not self.has_state):
            rospy.loginfo_throttle(
                2.0,
                "Waiting... has_odom=%s, has_state=%s",
                str(self.has_odom),
                str(self.has_state)
            )
            rate.sleep()

        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y
        start_z = self.odom.pose.pose.position.z
        start_yaw = self.yaw_from_quat(self.odom.pose.pose.orientation)

        rospy.loginfo(
            "Start pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
            start_x,
            start_y,
            start_z,
            start_yaw
        )

        # 1. 预发布 setpoint，满足 PX4 OFFBOARD 条件
        rospy.loginfo("Pre-publishing setpoints before OFFBOARD...")
        pre_start = rospy.Time.now()

        while not rospy.is_shutdown() and (rospy.Time.now() - pre_start).to_sec() < 2.0:
            self.publish_cmd(start_x, start_y, max(start_z, 0.2), start_yaw)
            rate.sleep()

        # 2. 请求 OFFBOARD
        if self.auto_offboard:
            rospy.loginfo("Trying to switch OFFBOARD...")

            for _ in range(10):
                if rospy.is_shutdown():
                    return

                if self.state.mode == "OFFBOARD":
                    rospy.loginfo("Already in OFFBOARD")
                    break

                self.call_offboard()

                wait_start = rospy.Time.now()
                while not rospy.is_shutdown() and (rospy.Time.now() - wait_start).to_sec() < 0.5:
                    self.publish_cmd(start_x, start_y, max(start_z, 0.2), start_yaw)
                    rate.sleep()

                rospy.loginfo("Current mode=%s", self.state.mode)

            if self.state.mode != "OFFBOARD":
                rospy.logerr("Failed to enter OFFBOARD. Abort takeoff.")
                return

        # 3. 解锁
        if self.auto_arm:
            rospy.loginfo("Trying to arm...")

            for _ in range(10):
                if rospy.is_shutdown():
                    return

                if self.state.armed:
                    rospy.loginfo("Already armed")
                    break

                self.call_arm()

                wait_start = rospy.Time.now()
                while not rospy.is_shutdown() and (rospy.Time.now() - wait_start).to_sec() < 0.5:
                    self.publish_cmd(start_x, start_y, max(start_z, 0.2), start_yaw)
                    rate.sleep()

                rospy.loginfo("Current armed=%s", str(self.state.armed))

            if not self.state.armed:
                rospy.logerr("Failed to arm. Abort takeoff.")
                return

        # 4. 平滑起飞
        rospy.loginfo("Taking off to z=%.2f", self.target_z)
        takeoff_start = rospy.Time.now()

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - takeoff_start).to_sec()
            ratio = max(0.0, min(1.0, t / self.takeoff_time))

            s = ratio * ratio * (3.0 - 2.0 * ratio)
            z_cmd = start_z + s * (self.target_z - start_z)

            self.publish_cmd(start_x, start_y, z_cmd, start_yaw)

            rospy.loginfo_throttle(
                1.0,
                "Taking off: z_cmd=%.2f, mode=%s, armed=%s",
                z_cmd,
                self.state.mode,
                str(self.state.armed)
            )

            if ratio >= 1.0:
                break

            rate.sleep()

        # 5. 起飞完成后持续悬停
        rospy.loginfo("Takeoff complete. Holding at z=%.2f", self.target_z)

        while not rospy.is_shutdown():
            self.publish_cmd(start_x, start_y, self.target_z, start_yaw)

            rospy.loginfo_throttle(
                2.0,
                "Holding: x=%.2f, y=%.2f, z=%.2f, mode=%s, armed=%s",
                start_x,
                start_y,
                self.target_z,
                self.state.mode,
                str(self.state.armed)
            )

            rate.sleep()


if __name__ == "__main__":
    node = AutoTakeoffGCS()
    node.run()
    
