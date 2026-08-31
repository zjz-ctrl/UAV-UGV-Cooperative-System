#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Header


def main():
    rospy.init_node("pub_cmd_wall_target", anonymous=False)

    topic = rospy.get_param("~topic", "/position_cmd_raw")
    rate_hz = rospy.get_param("~rate", 30.0)

    pub = rospy.Publisher(topic, PositionCommand, queue_size=10)
    rate = rospy.Rate(rate_hz)

    msg = PositionCommand()
    msg.header = Header()
    msg.header.frame_id = "map"

    # =============================
    # 🚩 关键：目标点在“墙后”
    # =============================
    msg.position.x = 5.0   # 墙在 1~2m，这里故意放在后面
    msg.position.y = 0.0
    msg.position.z = 1.0

    # 不给速度（或给很小），lookahead 会退化为“目标点方向”
    msg.velocity.x = 0.0
    msg.velocity.y = 0.0
    msg.velocity.z = 0.0

    # 其他项全部置零（交给 wrapper 决定 yaw）
    msg.acceleration.x = 0.0
    msg.acceleration.y = 0.0
    msg.acceleration.z = 0.0

    msg.jerk.x = 0.0
    msg.jerk.y = 0.0
    msg.jerk.z = 0.0

    # ⚠️ yaw/yaw_dot 在 raw 里不重要（会被 wrapper 覆盖）
    msg.yaw = 0.0
    msg.yaw_dot = 0.0

    rospy.loginfo(
        f"Publishing wall-behind target to {topic} at {rate_hz} Hz "
        "(target = [5,0,1])"
    )

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
