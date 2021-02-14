cmake_minimum_required(VERSION 3.0.2)

project(unity_connection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
  moveit_msgs
  sensor_msgs
  controller_manager
  roslaunch
  trajectory_msgs
)
roslaunch_add_file_check(launch)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  moveit_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/server_endpoint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)