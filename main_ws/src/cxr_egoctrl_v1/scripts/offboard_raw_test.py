#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool


class OffboardRawTest:
    def __init__(self):
        rospy.init_node("offboard_raw_test")

        self.state = State()

        self.state_sub = rospy.Subscriber(
            "/mavros/state",
            State,
            self.state_cb,
            queue_size=10
        )

        self.raw_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local",
            PositionTarget,
            queue_size=10
        )

        self.set_mode_client = rospy.ServiceProxy(
            "/mavros/set_mode",
            SetMode
        )

        self.arm_client = rospy.ServiceProxy(
            "/mavros/cmd/arming",
            CommandBool
        )

        self.target_z = rospy.get_param("~target_z", 1.0)
        self.auto_arm = rospy.get_param("~auto_arm", False)

        rospy.loginfo("offboard_raw_test started")
        rospy.loginfo("target_z = %.2f, auto_arm = %s", self.target_z, self.auto_arm)

    def state_cb(self, msg):
        self.state = msg

    def make_raw_setpoint(self):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"

        # MAV_FRAME_LOCAL_NED = 1
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # 使用 position + yaw
        # 忽略 velocity、acceleration、yaw_rate
        sp.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        sp.position.x = 0.0
        sp.position.y = 0.0
        sp.position.z = self.target_z

        sp.velocity.x = 0.0
        sp.velocity.y = 0.0
        sp.velocity.z = 0.0

        sp.acceleration_or_force.x = 0.0
        sp.acceleration_or_force.y = 0.0
        sp.acceleration_or_force.z = 0.0

        sp.yaw = 0.0
        sp.yaw_rate = 0.0

        return sp

    def wait_for_mavros(self):
        rate = rospy.Rate(10)

        rospy.loginfo("Waiting for MAVROS connection...")

        while not rospy.is_shutdown() and not self.state.connected:
            rospy.loginfo_throttle(
                1.0,
                "MAVROS not connected yet. mode=%s armed=%s",
                self.state.mode,
                self.state.armed
            )
            rate.sleep()

        rospy.loginfo("MAVROS connected. mode=%s armed=%s", self.state.mode, self.state.armed)

    def pre_publish_setpoints(self):
        rate = rospy.Rate(30)

        rospy.loginfo("Pre-publishing raw setpoints for 3 seconds...")

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - start_time).to_sec() > 3.0:
                break

            sp = self.make_raw_setpoint()
            self.raw_pub.publish(sp)
            rate.sleep()

        rospy.loginfo("Pre-publish done.")

    def request_offboard(self):
        rospy.wait_for_service("/mavros/set_mode", timeout=5.0)

        rospy.loginfo("Requesting OFFBOARD...")

        for i in range(10):
            resp = self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            rospy.loginfo("OFFBOARD request %d: mode_sent=%s", i + 1, resp.mode_sent)

            end_time = rospy.Time.now() + rospy.Duration(0.5)
            rate = rospy.Rate(30)

            while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                sp = self.make_raw_setpoint()
                self.raw_pub.publish(sp)
                rate.sleep()

            rospy.loginfo(
                "Current state: mode=%s armed=%s connected=%s",
                self.state.mode,
                self.state.armed,
                self.state.connected
            )

            if self.state.mode == "OFFBOARD":
                rospy.loginfo("Entered OFFBOARD successfully.")
                return True

        rospy.logerr("Failed to enter OFFBOARD.")
        return False

    def arm_if_needed(self):
        if not self.auto_arm:
            rospy.loginfo("auto_arm is false, skip arming.")
            return

        rospy.wait_for_service("/mavros/cmd/arming", timeout=5.0)

        rospy.loginfo("Requesting arm...")

        for i in range(5):
            resp = self.arm_client(True)
            rospy.loginfo("Arm request %d: success=%s result=%s", i + 1, resp.success, resp.result)

            if resp.success:
                return

            rospy.sleep(0.5)

    def spin_publish(self):
        rate = rospy.Rate(30)

        rospy.loginfo("Keep publishing raw setpoints. Press Ctrl+C to stop.")

        while not rospy.is_shutdown():
            sp = self.make_raw_setpoint()
            self.raw_pub.publish(sp)

            rospy.loginfo_throttle(
                1.0,
                "Publishing raw setpoint. mode=%s armed=%s target=(0,0,%.2f)",
                self.state.mode,
                self.state.armed,
                self.target_z
            )

            rate.sleep()

    def run(self):
        self.wait_for_mavros()
        self.pre_publish_setpoints()

        ok = self.request_offboard()

        if ok:
            self.arm_if_needed()

        self.spin_publish()


if __name__ == "__main__":
    node = OffboardRawTest()
    node.run()
