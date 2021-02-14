#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)

    # Load arm and gripper names
    arm_names = rospy.get_param("/ARM_NAMES")
    gripper_names = rospy.get_param("/GRIPPER_NAMES")
    object_names = rospy.get_param("/OBJECT_NAMES")
    for arm in arm_names:
        tcp_server.source_destination_dict[arm + '/joint_states'] = RosPublisher(arm+'/joint_states', JointState, queue_size=1)
        tcp_server.source_destination_dict[arm + '/controller_states'] = RosPublisher(arm+'/controller_states', Bool, queue_size=1)
        tcp_server.source_destination_dict[arm + '/joint_path_command'] = RosSubscriber(arm+'/joint_path_command', JointTrajectory, tcp_server)
    for gripper in gripper_names:
        tcp_server.source_destination_dict[gripper + '/joint_states'] = RosPublisher(gripper+'/joint_states', JointState, queue_size=1)
        tcp_server.source_destination_dict[gripper + '/joint_path_command'] = RosSubscriber(gripper+'/joint_path_command', Float32, tcp_server)

    for object_name in object_names:
        tcp_server.source_destination_dict[object_name] = RosPublisher(object_name, Pose, queue_size=1)

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
