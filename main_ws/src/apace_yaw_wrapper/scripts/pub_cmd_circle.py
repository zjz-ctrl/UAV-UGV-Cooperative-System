#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Header

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def main():
    rospy.init_node("pub_cmd_circle_face_center", anonymous=False)

    topic   = rospy.get_param("~topic", "/position_cmd_raw")
    rate_hz = float(rospy.get_param("~rate", 30.0))

    # ===== 圆轨迹参数 =====
    cx = float(rospy.get_param("~cx", 0.0))   # 圆心 x
    cy = float(rospy.get_param("~cy", 0.0))   # 圆心 y
    cz = float(rospy.get_param("~cz", 1.0))   # 圆心 z（也是飞行高度）
    R  = float(rospy.get_param("~R", 2.0))    # 半径（米）
    w  = float(rospy.get_param("~w", 0.3))    # 角速度（rad/s） 0.2~0.6都比较稳

    pub = rospy.Publisher(topic, PositionCommand, queue_size=10)
    rate = rospy.Rate(rate_hz)

    msg = PositionCommand()
    msg.header = Header()
    msg.header.frame_id = "map"

    t0 = rospy.Time.now().to_sec()
    rospy.loginfo(f"Publishing {topic} circle around ({cx},{cy},{cz}), R={R}, w={w}, rate={rate_hz}Hz. Nose -> center")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec() - t0
        theta = w * t  # 绕圈相位

        # ===== 位置：圆轨迹 =====
        x = cx + R * math.cos(theta)
        y = cy + R * math.sin(theta)
        z = cz

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        # ===== 速度：切向（可选，但建议给，lookahead/控制器更稳定）=====
        # x = cx + R cos(theta) -> dx/dt = -R w sin(theta)
        # y = cy + R sin(theta) -> dy/dt =  R w cos(theta)
        msg.velocity.x = -R * w * math.sin(theta)
        msg.velocity.y =  R * w * math.cos(theta)
        msg.velocity.z = 0.0

        # 其余置零
        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0
        msg.jerk.x = 0.0
        msg.jerk.y = 0.0
        msg.jerk.z = 0.0

        # ===== 关键：yaw 始终朝向圆心 =====
        # 从当前点指向圆心的方向向量： (cx - x, cy - y)
        yaw = math.atan2(cy - y, cx - x)      # 机头指向圆心
        yaw = wrap_pi(yaw)

        msg.yaw = yaw
        msg.yaw_dot = 0.0  # 让控制器按 yaw 跟踪即可（需要的话也可估算 yaw_dot）

        msg.header.stamp = now
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
