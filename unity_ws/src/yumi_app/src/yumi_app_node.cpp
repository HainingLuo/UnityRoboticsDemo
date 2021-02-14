#include "yumi_app.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yumi_app");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(3);
    spinner.start();
    
    // Create an instance of YumiApplication class
    YumiApplication yumi_app(node_handle);
    // Run Application
    yumi_app.runApplication();
    
    // Terminate program
    ROS_INFO("Application COMPLETED");
    spinner.stop();
    return 0;
}