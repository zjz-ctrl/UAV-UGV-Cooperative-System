#!/usr/bin/env python3
import math
import rospy
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

class SquareCmd:
    def __init__(self):
        rospy.init_node("square_position_cmd")

        self.pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=1)
        self.sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)

        # 当前无人机位置（ROS ENU）
        self.px = None
        self.py = None
        self.pz = None

        # ====== 正方形参数（以原点 0,0,1 为左下角）======
        self.z = 1.0
        self.side = 1.0  # 边长 2m（你可以改）
        self.cx = 0.0
        self.cy = 0.0

        # 四个角点： (0,0)->(1,0)->(1,1)->(0,1)->回到(0,0)
        self.waypoints = [
            (self.cx,             self.cy,             self.z),
            (self.cx + self.side, self.cy,             self.z),
            (self.cx + self.side, self.cy + self.side, self.z),
            (self.cx,             self.cy + self.side, self.z),
        ]

        # 到点判定阈值（越大越“容易判定到点”，一般 0.15~0.30）
        self.reach_xy = 0.20
        self.reach_z  = 0.25

        # 每个点最多等待多少秒，防止卡死
        self.timeout_per_wp = 20.0

        self.yaw = 0.0
        self.yaw_dot = 0.0

    def odom_cb(self, msg: Odometry):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.pz = msg.pose.pose.position.z

    def dist_ok(self, tx, ty, tz):
        if self.px is None:
            return False
        dxy = math.hypot(tx - self.px, ty - self.py)
        dz  = abs(tz - self.pz)
        return (dxy < self.reach_xy) and (dz < self.reach_z)

    def publish_target(self, tx, ty, tz):
        m = PositionCommand()
        m.position.x = tx
        m.position.y = ty
        m.position.z = tz

        # 下面这些字段你代码里虽然不一定用，但填 0 更干净
        m.velocity.x = 0.0
        m.velocity.y = 0.0
        m.velocity.z = 0.0
        m.acceleration.x = 0.0
        m.acceleration.y = 0.0
        m.acceleration.z = 0.0

        m.yaw = self.yaw
        m.yaw_dot = self.yaw_dot
        self.pub.publish(m)

    def run(self):
        rate = rospy.Rate(20)  # /position_cmd 持续 20Hz 发就够稳了

        # 等待里程计
        rospy.loginfo("Waiting for /mavros/local_position/odom ...")
        while not rospy.is_shutdown() and self.px is None:
            rate.sleep()
        rospy.loginfo("Odom OK. Start square waypoints.")

        wp_idx = 0
        while not rospy.is_shutdown():
            tx, ty, tz = self.waypoints[wp_idx]
            rospy.loginfo("Go WP%d: (%.2f, %.2f, %.2f)", wp_idx, tx, ty, tz)

            start_t = rospy.Time.now().to_sec()
            # 在“到达前”，一直发布同一个目标点（最稳）
            while not rospy.is_shutdown():
                self.publish_target(tx, ty, tz)

                if self.dist_ok(tx, ty, tz):
                    rospy.loginfo("Reached WP%d", wp_idx)
                    break

                if rospy.Time.now().to_sec() - start_t > self.timeout_per_wp:
                    rospy.logwarn("Timeout at WP%d, switch next anyway.", wp_idx)
                    break

                rate.sleep()

            # 切换到下一个点
            wp_idx = (wp_idx + 1) % len(self.waypoints)
            rospy.sleep(0.5)  # 角点给一点点缓冲，更稳

if __name__ == "__main__":
    SquareCmd().run()
