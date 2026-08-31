#!/usr/bin/env python3
import rospy
from quadrotor_msgs.msg import PositionCommand as YopoCmd
from quadrotor_msgs.msg import PositionCommand as CxrCmd

pub = None

def callback(msg):
    out = CxrCmd()

    out.header = msg.header
    out.position = msg.position
    out.velocity = msg.velocity
    out.acceleration = msg.acceleration

    # YOPO 没有 jerk → 填0
    out.jerk.x = 0.0
    out.jerk.y = 0.0
    out.jerk.z = 0.0

    out.yaw = msg.yaw
    out.yaw_dot = msg.yaw_dot
    out.trajectory_id = msg.trajectory_id
    out.trajectory_flag = msg.trajectory_flag

    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("yopo_cxr_bridge")

    rospy.Subscriber("/position_cmd_yopo", YopoCmd, callback)
    pub = rospy.Publisher("/position_cmd", CxrCmd, queue_size=10)

    rospy.spin()
