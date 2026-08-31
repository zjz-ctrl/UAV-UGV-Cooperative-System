#!/usr/bin/env python3
import rospy
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Header

def main():
    rospy.init_node("pub_cmd_forward", anonymous=False)
    topic = rospy.get_param("~topic", "/position_cmd_raw")
    rate_hz = rospy.get_param("~rate", 30.0)

    pub = rospy.Publisher(topic, PositionCommand, queue_size=10)
    rate = rospy.Rate(rate_hz)

    msg = PositionCommand()
    msg.header = Header()
    msg.header.frame_id = "map"

    msg.position.x = 0.0
    msg.position.y = 0.0
    msg.position.z = 1.0

    # 给一个速度方向（lookahead会让yaw对准它）
    msg.velocity.x = 1.0
    msg.velocity.y = 1.0
    msg.velocity.z = 0.0

    rospy.loginfo(f"Publishing {topic} forward vel at {rate_hz} Hz")
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
