#include "yumi_app.h"

// ROS_INFO("..............................T E S T..................................");

YumiApplication::YumiApplication(const ros::NodeHandle node_handle) :
_left_arm_group("left_arm"), 
_right_arm_group("right_arm"), 
_both_arm_group("both_arms"),
_nh(node_handle)
{
    right_gripper_cmd_pub = _nh.advertise<std_msgs::Float32>("/yumi/right_gripper/joint_path_command", 1);
    left_gripper_cmd_pub = _nh.advertise<std_msgs::Float32>("/yumi/left_gripper/joint_path_command", 1);
    target_sub = _nh.subscribe<geometry_msgs::Pose>("/target", 10, &YumiApplication::targetCallback, this);
    target_placement_sub = _nh.subscribe<geometry_msgs::Pose>("/target_placement", 10, &YumiApplication::targetPlacementCallback, this);
}

void YumiApplication::runApplication()
{
    closeLeftGripper();
    closeRightGripper();
    goInit();
    
    Vec3f ori = Vec3f(0, M_PI, 0);
    Vec3f tar = target + Vec3f(0, 0, 0.2);
    leftGoTo(tar, ori, "Approaching");
    openLeftGripper();
    tar = target + Vec3f(0, 0, 0.13);
    leftGoTo(tar, ori, "Engaging");
    closeLeftGripper();
    tar = target + Vec3f(0, 0, 0.2);
    leftGoTo(tar, ori, "Retreating");
    tar = target_placement + Vec3f(0, 0, 0.2);
    leftGoTo(tar, ori, "Transporting");
    tar = target_placement + Vec3f(0, 0, 0.13);
    leftGoTo(tar, ori, "Placing");
    openLeftGripper();
    tar = target_placement + Vec3f(0, 0, 0.2);
    leftGoTo(tar, ori, "Retreating");
    closeLeftGripper();

    // Send the arms back to original positions
    goHome();
}

// Send the arm to target position and return the position of the marker
void YumiApplication::leftGoTo(const Vec3f& target, const Vec3f& orientation, const std::string& name)
{
    _left_arm_group.setStartState(*_left_arm_group.getCurrentState());
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::Pose target_pose;
    bool success=false, executed=false;
    char cstr[name.size() + 1];
    strcpy(cstr, name.c_str());
    
    tf2::Quaternion quat;
    quat.setRPY(orientation(0), orientation(1), orientation(2));
    target_pose.orientation = tf2::toMsg(quat);
    target_pose.position.x = target(0);
    target_pose.position.y = target(1);
    target_pose.position.z = target(2);
    _left_arm_group.setPoseTarget(target_pose);

    success = _left_arm_group.plan(plan) == moveit_msgs::MoveItErrorCodes::SUCCESS;
    // std::cout << plan.trajectory_.joint_trajectory.joint_names << std::endl;
    ROS_INFO("[%s] Planning %s acheived. Start Execution? y/n" ,cstr ,success? "SUCCESS" : "FAILED");
    //if planning succeeded, execute the plan    
    if(checkCommand()) {
        executed = _left_arm_group.execute(plan) == moveit_msgs::MoveItErrorCodes::SUCCESS;
        ROS_INFO("[%s] Executing plan: %s" ,cstr ,executed ? "SUCCESS" : "FAILED");
	    _left_arm_group.setStartStateToCurrentState();
    }
    else {
	    ROS_WARN("[%s] Action Skipped" ,cstr);
    }
}

// Send the arm to the initial position for detection test
void YumiApplication::goInit()
{   
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success=false, executed=false;
    double progress=0;
    
    _both_arm_group.setNamedTarget("home");
     
    success = _both_arm_group.plan(plan) == moveit_msgs::MoveItErrorCodes::SUCCESS;
    ROS_INFO("[GO INIT] Planning %s. Start Execution? y/n" ,success? "SUCCESS" : "FAILED");
    
    //if planning succeeded, execute the plan
    if(checkCommand())
    {
        executed = _left_arm_group.execute(plan) == moveit_msgs::MoveItErrorCodes::SUCCESS;
        ROS_INFO("[GO INIT] Execution: %s" ,executed ? "SUCCESS" : "FAILED");
        _left_arm_group.setStartStateToCurrentState();
    }
    else {
	    ROS_WARN("[GO INIT] Action Skipped");
    }
}

// go back to the home position
void YumiApplication::goHome()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success=false, executed=false;
    double progress=0;
    
    _both_arm_group.setNamedTarget("calc");
     
    success = _both_arm_group.plan(plan) == moveit_msgs::MoveItErrorCodes::SUCCESS;
    ROS_INFO("[GO HOME] Planning %s. Start Execution? y/n" ,success? "SUCCESS" : "FAILED");
    
    //if planning succeeded, execute the plan   
    if(checkCommand())
    {
        executed = _left_arm_group.execute(plan) == moveit_msgs::MoveItErrorCodes::SUCCESS;
        ROS_INFO("[GO HOME] Execution: %s" ,executed ? "SUCCESS" : "FAILED");
        _left_arm_group.setStartStateToCurrentState();
    }
    else {
	    ROS_WARN("[GO HOME] Action Skipped");
    }
}