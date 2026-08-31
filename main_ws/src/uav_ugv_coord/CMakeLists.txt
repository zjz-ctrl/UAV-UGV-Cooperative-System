cmake_minimum_required(VERSION 3.0.2)
project(uav_ugv_coord)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2_ros
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  src/tag_detector_node.py
  src/coord_merge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_coord_merge.py)
endif()
