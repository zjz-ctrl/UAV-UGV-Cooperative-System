cmake_minimum_required(VERSION 3.0.2)
project(air_ground_coordinate_transform)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_python_setup()

add_message_files(
  FILES
  RegistrationUpdate.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp std_msgs tf tf2 tf2_geometry_msgs tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(coordinate_transform_node src/coordinate_transform_node.cpp)
target_link_libraries(coordinate_transform_node ${catkin_LIBRARIES})

install(TARGETS coordinate_transform_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/takeoff_registration.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_se2.py)
  catkin_add_nosetests(test/test_odom_buffer.py)
  catkin_add_nosetests(test/test_acquisition_diagnostics.py)
  catkin_add_nosetests(test/test_registration_estimator.py)
  catkin_add_nosetests(test/test_registration_coordinator.py)
  catkin_add_nosetests(test/test_registration_node_adapter.py)
  catkin_add_nosetests(test/test_registration_update_serialization.py)
  catkin_add_nosetests(test/test_ugv_coordinate_monitor.py)
  add_rostest(test/registration_node.test)
endif()
