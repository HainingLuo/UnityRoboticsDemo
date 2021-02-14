using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System;
using RosMessageTypes.Geometry;
using UnityEngine;
using RosQuaternion = RosMessageTypes.Geometry.Quaternion;
using Transform = UnityEngine.Transform;
using RosSharp.Urdf;

[RequireComponent(typeof(ROSConnection))]
public class ArmController : MonoBehaviour
{
    private static Timer timer = new Timer();

    // Define variables 
    private readonly float jointAssignmentWait = 0.1f;
    private float publish_rate_control = 0;
    private float update_time_step;
    private int numRobotJoints;

    // ROS Connector
    private ROSConnection rosConnector;
    
    // ROS communication related variables
    public string armName;
    private string pathTopicName;
    private string controllerStateTopicName;
    private string jointStateTopicName;
    private string frame_id = "";
    private List<string> joint_names = new List<string>();

    private bool controller_states = false;
    private RosMessageTypes.Std.Bool controller_state_msg;
    private RosMessageTypes.Sensor.JointState joint_state_msg;

    // Unity Objects
    public GameObject arm;
    private List<ArticulationBody> jointArticulationBodies = new List<ArticulationBody>();

    void TrajectoryCallback(RosMessageTypes.Trajectory.JointTrajectory trajectory)
    {
        if (trajectory != null)
        {
            // Debug.Log("Trajectory received.");
            StartCoroutine(ExecuteTrajectories(trajectory));
            controller_states = true;
        }
        else
        {
            // *****************************************
            // Empty trajectory, cancel current execution. 
            // (The trajectory has already been executed under current implementation,
            // skipping this fornow.)
            // *****************************************
            // for (int i=0; i<jointArticulationBodies.Count; i++) {
            //     var drive = jointArticulationBodies[i].xDrive;
            //     drive.target = jointArticulationBodies[i].jointPosition[0];
            //     jointArticulationBodies[i].xDrive = drive;
            // }
        }
    }
    
    private IEnumerator ExecuteTrajectories(RosMessageTypes.Trajectory.JointTrajectory trajectory)
    {
        // For every robot pose in trajectory plan
        for (int jointConfigIndex  = 0 ; jointConfigIndex < trajectory.points.Length; jointConfigIndex++)
        {
            var jointPositions = trajectory.points[jointConfigIndex].positions;
            float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
            
            // Set the joint values for every joint
            for (int joint = 0; joint < jointArticulationBodies.Count; joint++)
            {
                int joint_index = joint_names.IndexOf(trajectory.joint_names[joint]);
                var joint1XDrive = jointArticulationBodies[joint_index].xDrive;
                joint1XDrive.target = result[joint];
                jointArticulationBodies[joint_index].xDrive = joint1XDrive;
            }
            // Wait for robot to achieve pose for all joint assignments
            yield return new WaitForSeconds(jointAssignmentWait);
        }
        controller_states = false;
        controller_state_msg.data = controller_states;
        // publish the message
        rosConnector.Send(controllerStateTopicName, controller_state_msg);
    }

    // Initialise joint state messages
    private void InitializeMessage()
    {
        controller_state_msg = new RosMessageTypes.Std.Bool(false);

        joint_state_msg = new RosMessageTypes.Sensor.JointState
        {
            header = new RosMessageTypes.Std.Header { frame_id = frame_id },
            name = joint_names.ToArray(),
            position = new double[numRobotJoints],
            velocity = new double[numRobotJoints],
            effort = new double[numRobotJoints]
        };
    }

    // publish joint states
    private void FixedUpdate()
    {
        if (publish_rate_control >= 0.1) // control the publish rate at 10 fps
        {
            publish_rate_control = 0;
            joint_state_msg.header.seq ++;
            timer.Now(joint_state_msg.header.stamp);
            for (int i = 0; i < numRobotJoints; i++) {
                joint_state_msg.position[i] = jointArticulationBodies[i].jointPosition[0];
                joint_state_msg.velocity[i] = jointArticulationBodies[i].jointVelocity[0];
                joint_state_msg.effort[i] = jointArticulationBodies[i].jointAcceleration[0];
            }
            rosConnector.Send(jointStateTopicName, joint_state_msg);
        }
        else 
        {
            publish_rate_control += update_time_step;
        }
    }

    void Awake()
    {
        pathTopicName = armName + "/joint_path_command";
        controllerStateTopicName = armName + "/controller_states";
        jointStateTopicName = armName + "/joint_states";

        rosConnector = GetComponent<ROSConnection>();
        // Extract all articulation bodies with revolute joints 
        UrdfJointRevolute[] revoluteJoints = arm.GetComponentsInChildren<UrdfJointRevolute>();
        for (int i=0; i<revoluteJoints.Length; i++) {
            jointArticulationBodies.Add(revoluteJoints[i].gameObject.GetComponent<ArticulationBody>());
            joint_names.Add(revoluteJoints[i].jointName);
            // Debug.Log("Number: " + joint_names[i]);
        }
        // Debug.Log("Number: " + jointArticulationBodies.Count);
        numRobotJoints = revoluteJoints.Length;

        update_time_step = Time.fixedDeltaTime;
        InitializeMessage();

        rosConnector.Subscribe<RosMessageTypes.Trajectory.JointTrajectory>(pathTopicName, TrajectoryCallback);
    }

}
