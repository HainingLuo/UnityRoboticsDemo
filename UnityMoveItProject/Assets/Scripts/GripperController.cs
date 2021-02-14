using System.Collections.Generic;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using UnityEngine;
using RosQuaternion = RosMessageTypes.Geometry.Quaternion;
using Transform = UnityEngine.Transform;
using RosSharp.Urdf;

[RequireComponent(typeof(ROSConnection))]
public class GripperController : MonoBehaviour
{
    private static Timer timer = new Timer();

    //Define Variables
    private readonly float jointAssignmentWait = 0.1f;
    private float publish_rate_control = 0;
    private float update_time_step;
    private int numGripperJoints;

    // ROS Connector
    private ROSConnection rosConnector;

    // ROS communication related variables
    public string gripperName;
    private string pathTopicName;
    private string jointStateTopicName;
    private string frame_id = "";
    private List<string> joint_names = new List<string>();

    private RosMessageTypes.Sensor.JointState left_joint_state_msg;
    private RosMessageTypes.Sensor.JointState right_joint_state_msg;
    private RosMessageTypes.Sensor.JointState gripper_state_msg;

    // Unity Objects
    public GameObject gripper;
    private List<ArticulationBody> jointArticulationBodies = new List<ArticulationBody>();

    private IEnumerator gripperTo(float percentage)
    {
        for (int i=0; i<jointArticulationBodies.Count; i++) {
            var drive = jointArticulationBodies[i].xDrive;
            drive.target = drive.upperLimit * percentage; // may need to modify this for other grippers
            jointArticulationBodies[i].xDrive = drive;
        }
        yield return new WaitForSeconds(jointAssignmentWait);
    }
    
    public void CloseGripper() {StartCoroutine(gripperTo(0));}
    public void OpenGripper() {StartCoroutine(gripperTo(1));}

    void TrajectoryCallback(RosMessageTypes.Std.Float32 trajectory)
    {
        if (trajectory != null)
        {
            // Debug.Log("Trajectory received.");
            StartCoroutine(gripperTo(trajectory.data));
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
    
    // Initialise joint state messages
    private void InitializeMessage()
    {
        gripper_state_msg = new RosMessageTypes.Sensor.JointState
        {
            header = new RosMessageTypes.Std.Header { frame_id = frame_id },
            name = joint_names.ToArray(),
            position = new double[numGripperJoints],
            velocity = new double[numGripperJoints],
            effort = new double[numGripperJoints]
        };
    }

    // publish joint states
    private void FixedUpdate()
    {
        if (publish_rate_control >= 0.1) // control the publish rate at 10 fps
        {
            publish_rate_control = 0;

            // Grippers
            gripper_state_msg.header.seq ++;
            timer.Now(gripper_state_msg.header.stamp);
            for (int i = 0; i < numGripperJoints; i++) {
                gripper_state_msg.position[i] = jointArticulationBodies[i].jointPosition[0];
                gripper_state_msg.velocity[i] = jointArticulationBodies[i].jointVelocity[0];
                gripper_state_msg.effort[i] = jointArticulationBodies[i].jointAcceleration[0];
            }
            rosConnector.Send(jointStateTopicName, gripper_state_msg);
        }
        else 
        {
            publish_rate_control += update_time_step;
        }
    }

    void Awake()
    {
        pathTopicName = gripperName + "/joint_path_command";
        jointStateTopicName = gripperName + "/joint_states";

        rosConnector = GetComponent<ROSConnection>();
        // Extract all articulation bodies with primatic joints 
        UrdfJointPrismatic[] prismaticJoints = gripper.GetComponentsInChildren<UrdfJointPrismatic>();
        for (int i=0; i<prismaticJoints.Length; i++) {
            jointArticulationBodies.Add(prismaticJoints[i].gameObject.GetComponent<ArticulationBody>());
            joint_names.Add(prismaticJoints[i].jointName);
            // Debug.Log("Joint Names: " + joint_names[i]);
        }
        numGripperJoints = prismaticJoints.Length;

        update_time_step = Time.fixedDeltaTime;
        InitializeMessage();

        // Close grippers by default
        StartCoroutine(gripperTo(0));

        rosConnector.Subscribe<RosMessageTypes.Std.Float32>(pathTopicName, TrajectoryCallback);
    }

}
