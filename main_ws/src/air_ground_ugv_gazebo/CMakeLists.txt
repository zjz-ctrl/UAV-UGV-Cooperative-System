cmake_minimum_required(VERSION 3.0.2)
project(air_ground_ugv_gazebo)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(relative_pose_sensor_plugin src/relative_pose_sensor_plugin.cpp)
target_link_libraries(relative_pose_sensor_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS relative_pose_sensor_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
