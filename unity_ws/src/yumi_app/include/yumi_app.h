#ifndef Yumi_HAND_EYE_CALIB_H
#define Yumi_HAND_EYE_CALIB_H

#include <iostream>
#include <math.h>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <std_msgs/Float32.h>
#include "geometry_msgs/PointStamped.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

#include "Eigen/Dense"

typedef moveit::planning_interface::MoveGroupInterface MoveGroup;
typedef Eigen::Vector3f Vec3f;
typedef geometry_msgs::Pose Pose;
typedef moveit_msgs::RobotTrajectory RobotTrajectory;

class YumiApplication
{

public:
    YumiApplication(const ros::NodeHandle node_handle);
    void runApplication();
    
private:
    
    bool auto_execution=false;
    ros::NodeHandle _nh;
    ros::Publisher display_publisher;
    moveit_msgs::DisplayTrajectory display_trajectory;
    ros::Publisher right_gripper_cmd_pub, left_gripper_cmd_pub;
    ros::Subscriber target_sub, target_placement_sub;

    moveit::planning_interface::MoveGroupInterface _left_arm_group, _right_arm_group, _both_arm_group;
    moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;
    
    Vec3f target, target_placement;

    void goInit();
    void goHome();

    void leftGoTo(const Vec3f& target, const Vec3f& orientation, const std::string& name);

    // Gripper
    // Close the right gripper
    void closeRightGripper() { sendRightGripperCmd(std::abs(0)); }
    // Open the right gripper
    void openRightGripper() { sendRightGripperCmd(std::abs(1)); }
    // Close the left gripper
    void closeLeftGripper() { sendLeftGripperCmd(std::abs(0)); }
    // Open the right gripper
    void openLeftGripper() { sendLeftGripperCmd(std::abs(1)); }

    void sendRightGripperCmd(const double& pos_cmd) {
        std_msgs::Float32 msg;
        msg.data = pos_cmd;
        right_gripper_cmd_pub.publish(msg);
        ros::Duration(1.0).sleep();
    }

    void sendLeftGripperCmd(const double& pos_cmd) {
        std_msgs::Float32 msg;
        msg.data = pos_cmd;
        left_gripper_cmd_pub.publish(msg);
        ros::Duration(1.0).sleep();
    }

    void targetCallback(const geometry_msgs::Pose msg) {
        target(0) = msg.position.x;
        target(1) = msg.position.y;
        target(2) = msg.position.z;
    }

    void targetPlacementCallback(const geometry_msgs::Pose msg) {
        target_placement(0) = msg.position.x;
        target_placement(1) = msg.position.y;
        target_placement(2) = msg.position.z;
    }

    // Utility
    // Check if y is pressed
    bool checkCommand()
    {
        if (auto_execution) return 1;
        else {
        std::string command;
        std::cin >> command;
        return (command == "y" ? 1 : 0);
        }
    }
};

#endif