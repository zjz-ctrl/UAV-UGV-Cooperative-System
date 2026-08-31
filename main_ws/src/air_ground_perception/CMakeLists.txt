cmake_minimum_required(VERSION 3.0.2)
project(air_ground_perception)
find_package(catkin REQUIRED COMPONENTS cv_bridge geometry_msgs rospy sensor_msgs std_msgs visualization_msgs)
catkin_package(CATKIN_DEPENDS cv_bridge geometry_msgs rospy sensor_msgs std_msgs visualization_msgs)
catkin_install_python(PROGRAMS
  scripts/charuco_detector.py
  scripts/nadir_red_sphere_detector.py
  scripts/red_sphere_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
