cmake_minimum_required(VERSION 3.0.2)
project(air_ground_bringup)

find_package(catkin REQUIRED COMPONENTS
  air_ground_coordinate_transform
  air_ground_experiments
  air_ground_perception
  mavros_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  std_msgs
  tf
  visualization_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS air_ground_coordinate_transform air_ground_experiments air_ground_perception geometry_msgs mavros_msgs nav_msgs quadrotor_msgs rospy std_msgs tf visualization_msgs)

catkin_install_python(PROGRAMS
  scripts/auto_takeoff_trigger.py
  scripts/camera_mosaic.py
  scripts/relative_target_estimator.py
  scripts/ugv_patrol.py
  scripts/uav_follow_mission.py
  scripts/uav_sphere_mission.py
  scripts/ugv_coordinate_monitor.py
  scripts/ugv_goal_controller.py
  scripts/world_boundary_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_auto_takeoff_compat.py)
  catkin_add_nosetests(test/test_launch_wiring.py)
  catkin_add_nosetests(test/test_offboard_compat.py)
  catkin_add_nosetests(test/test_target_handoff.py)
  catkin_add_nosetests(test/test_reregistration_state_machine.py)
  catkin_add_nosetests(test/test_setup_mvp_env.py)
  catkin_add_nosetests(test/test_ugv_goal_controller.py)
endif()
