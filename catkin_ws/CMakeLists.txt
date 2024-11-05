cmake_minimum_required(VERSION 3.0.2)
project(object_tracking)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
