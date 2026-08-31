cmake_minimum_required(VERSION 3.0.2)
project(air_ground_experiments)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  std_msgs
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS gazebo_msgs geometry_msgs nav_msgs quadrotor_msgs rospy std_msgs tf
)

catkin_install_python(PROGRAMS
  scripts/odom_perturbation_node.py
  scripts/observation_gate.py
  scripts/position_command_adapter.py
  scripts/experiment_recorder.py
  scripts/run_m2c_dynamic_validation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_frame_perturbation.py)
  catkin_add_nosetests(test/test_metrics.py)
  catkin_add_nosetests(test/test_adapter_serialization.py)
  catkin_add_nosetests(test/test_package_safety.py)
endif()
