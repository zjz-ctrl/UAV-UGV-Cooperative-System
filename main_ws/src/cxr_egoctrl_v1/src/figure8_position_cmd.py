#!/usr/bin/env python3
import math
import rospy
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry


class Figure8Cmd:
    def __init__(self):
        rospy.init_node("figure8_position_cmd")

        self.pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=1)
        self.sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)

        # 当前无人机位置（ENU）
        self.px = None
        self.py = None
        self.pz = None

        # ========== 轨迹参数（低速、稳定优先） ==========
        self.z = 1.2              # 飞行高度
        self.a = 0.6              # 8字在 x 方向半幅（建议 0.8~1.2）
        self.b = 1.2              # 8字在 y 方向半幅（建议 0.4~0.8）
        self.w = 0.30             # 角速度，越小越慢越稳（建议 0.16~0.25）

        # 起飞后先悬停几秒，再开始轨迹
        self.hover_time = 3.0

        # 轨迹渐入时间：避免突然切大轨迹
        self.ramp_time = 5.0

        # 固定朝向，先不要边飞边转头
        self.yaw = 0.0
        self.yaw_dot = 0.0

        # 轨迹中心，启动后锁定为当前悬停点
        self.cx = 0.0
        self.cy = 0.0

    def odom_cb(self, msg: Odometry):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.pz = msg.pose.pose.position.z

    def publish_target(self, tx, ty, tz):
        msg = PositionCommand()

        msg.position.x = tx
        msg.position.y = ty
        msg.position.z = tz

        # 不强行给速度前馈，优先求稳
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0

        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0

        msg.yaw = self.yaw
        msg.yaw_dot = self.yaw_dot

        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(30)   # 30Hz 连续发目标点更平滑

        rospy.loginfo("Waiting for /mavros/local_position/odom ...")
        while not rospy.is_shutdown() and self.px is None:
            rate.sleep()
        rospy.loginfo("Odom OK.")

        # 把当前点作为8字中心，先原地悬停稳住
        self.cx = self.px
        self.cy = self.py

        rospy.loginfo("Hover for %.1f s at current position: (%.2f, %.2f, %.2f)",
                      self.hover_time, self.cx, self.cy, self.z)

        hover_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - hover_start >= self.hover_time:
                break
            self.publish_target(self.cx, self.cy, self.z)
            rate.sleep()

        rospy.loginfo("Start low-speed figure-8 trajectory.")
        traj_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - traj_start

            # 幅度渐入，避免一开始横向突变
            scale = min(1.0, t / self.ramp_time)

            # 低速平滑 8 字轨迹
            # x = a * sin(w t)
            # y = b * sin(w t) * cos(w t) = 0.5*b*sin(2wt)
            tx = self.cx + scale * self.a * math.sin(2.0 * self.w * t)
            ty = self.cy + scale * self.b * math.sin(self.w * t)
            tz = self.z

            self.publish_target(tx, ty, tz)
            rate.sleep()


if __name__ == "__main__":
    try:
        Figure8Cmd().run()
    except rospy.ROSInterruptException:
        pass