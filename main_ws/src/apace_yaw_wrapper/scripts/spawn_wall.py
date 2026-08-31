#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

WALL_SDF = r"""
<sdf version="1.6">
  <model name="apace_wall">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box><size>0.1 4.0 2.0</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.1 4.0 2.0</size></box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

def main():
    rospy.init_node("spawn_apace_wall")

    x = rospy.get_param("~x", 1.5)   # 墙中心放在机体前方 1.5m
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 1.0)   # 墙中心高度 1m（墙高 2m）

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    model_name = "apace_wall"
    resp = spawn(model_name, WALL_SDF, "", pose, "world")
    rospy.loginfo("spawn result: %s", resp.status_message)

if __name__ == "__main__":
    main()
