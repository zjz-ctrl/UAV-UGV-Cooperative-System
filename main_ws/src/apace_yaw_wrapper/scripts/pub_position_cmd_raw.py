#!/usr/bin/env python3
import rospy
import math

from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Header

def main():
    rospy.init_node("pub_position_cmd_raw", anonymous=False)

    topic = rospy.get_param("~topic", "/position_cmd_raw")
    rate_hz = rospy.get_param("~rate", 30.0)

    pub = rospy.Publisher(topic, PositionCommand, queue_size=10)
    rate = rospy.Rate(rate_hz)

    msg = PositionCommand()
    msg.header = Header()
    msg.header.frame_id = "map"

    # 固定位置：原地悬停
    msg.position.x = 0.0
    msg.position.y = 0.0
    msg.position.z = 1.0

    # 速度 / 加速度 / jerk 置零
    msg.velocity.x = 0.0
    msg.velocity.y = 0.0
    msg.velocity.z = 0.0

    msg.acceleration.x = 0.0
    msg.acceleration.y = 0.0
    msg.acceleration.z = 0.0

    msg.jerk.x = 0.0
    msg.jerk.y = 0.0
    msg.jerk.z = 0.0

    # === 关键：yaw 原地转圈 ===
    yaw_rate = 0.6   # rad/s，转得慢一点更稳
    yaw = 0.0
    t0 = rospy.Time.now().to_sec()

    rospy.loginfo(f"Publishing {topic} at {rate_hz} Hz, yaw_rate={yaw_rate} rad/s")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec()
        yaw = yaw_rate * t

        # 关键：wrap到[-pi, pi]
        yaw = (yaw + math.pi) % (2*math.pi) - math.pi

        msg.header.stamp = now
        msg.yaw = yaw
        msg.yaw_dot = yaw_rate
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
