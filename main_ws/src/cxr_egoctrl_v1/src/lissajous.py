#!/usr/bin/env python3
import math
import rospy
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

class RaptorStyleLissajousCmd:
    def __init__(self):
        rospy.init_node("raptor_style_lissajous_cmd")

        self.pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=1)
        self.sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)

        self.px = None
        self.py = None
        self.pz = None

        # 轨迹中心高度
        self.z0 = 1.2

        # 更接近 RAPTOR figure-eight 的结构
        self.A = 0.4  # x 幅度（前后）
        self.B = 1.2  # y 幅度（左右）
        self.C = 0.0   # z 起伏，先关掉更稳

        self.a = 2.0
        self.b = 1.0
        self.c = 1.0

        self.interval = 16.0   # 一圈总时长，越大越慢
        self.ramp = 3.5        # 渐入时间

        self.hover_time = 3.0
        self.yaw = 0.0
        self.yaw_dot = 0.0

        self.cx = 0.0
        self.cy = 0.0

    def odom_cb(self, msg):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.pz = msg.pose.pose.position.z

    def publish_target(self, x, y, z, vx, vy, vz):
        msg = PositionCommand()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        # 这里给速度前馈，比全 0 更接近 RAPTOR 网站轨迹定义
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz

        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0

        msg.yaw = self.yaw
        msg.yaw_dot = self.yaw_dot
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(30)

        rospy.loginfo("Waiting for odom...")
        while not rospy.is_shutdown() and self.px is None:
            rate.sleep()

        self.cx = self.px
        self.cy = self.py

        # 先悬停
        hover_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - hover_start >= self.hover_time:
                break
            self.publish_target(self.cx, self.cy, self.z0, 0.0, 0.0, 0.0)
            rate.sleep()

        traj_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - traj_start

            # 按 RAPTOR 网站风格构造 progress
            time_velocity = min(t, self.ramp) / self.ramp if self.ramp > 0 else 1.0
            ramp_time = time_velocity * min(t, self.ramp) / 2.0
            progress = (ramp_time + max(0.0, t - self.ramp)) * 2.0 * math.pi / self.interval
            d_progress = 2.0 * math.pi * time_velocity / self.interval

            x = self.cx + self.A * math.sin(self.a * progress)
            y = self.cy + self.B * math.sin(self.b * progress)
            z = self.z0 + self.C * math.sin(self.c * progress)

            vx = self.A * math.cos(self.a * progress) * self.a * d_progress
            vy = self.B * math.cos(self.b * progress) * self.b * d_progress
            vz = self.C * math.cos(self.c * progress) * self.c * d_progress

            self.publish_target(x, y, z, vx, vy, vz)
            rate.sleep()

if __name__ == "__main__":
    try:
        RaptorStyleLissajousCmd().run()
    except rospy.ROSInterruptException:
        pass