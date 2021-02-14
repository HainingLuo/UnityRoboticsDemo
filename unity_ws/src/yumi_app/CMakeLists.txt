cmake_minimum_required(VERSION 2.8.3)
project(yumi_app)

# C++ 11 required
add_compile_options(-std=c++11 -O3)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
#     interactive_markers
#     rviz_visual_tools
    moveit_core
    moveit_ros_perception
    moveit_ros_planning
    moveit_ros_planning_interface
#     moveit_visual_tools
#     pluginlib
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    message_filters
)

find_package(Eigen3)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
	${PROJECT_NAME}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
    src/yumi_app.cpp
)

set(LIBS
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_lib
)

add_executable(${PROJECT_NAME}_node src/yumi_app_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${LIBS}
)