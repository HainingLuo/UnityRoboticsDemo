using UnityEngine;
using System.Collections.Generic;
using Point = RosMessageTypes.Geometry.Point;
using RosQuaternion = RosMessageTypes.Geometry.Quaternion;
using RosSharp.Urdf;
using RosTrans = RosSharp.TransformExtensions;

[RequireComponent(typeof(ROSConnection))]
public class ObjectStatePublisher : MonoBehaviour
{
    private static Timer timer = new Timer();

    private float publish_rate_control = 0;
    private float update_time_step;

    public string Topic;
    public GameObject target;

    private ROSConnection rosConnector;
    private RosMessageTypes.Geometry.Pose message;
    
    protected void Start()
    {
        rosConnector = GetComponent<ROSConnection>();
        update_time_step = Time.fixedDeltaTime;
        InitializeMessage();
    }

    private void InitializeMessage()
    {
        message = new RosMessageTypes.Geometry.Pose
        {
            position = new Point(0,0,0),
            orientation = new RosQuaternion(0, 1, 0, 0)
        };
    }

    private void FixedUpdate()
    {
        if (publish_rate_control >= 0.1) // control the publish rate at 10 fps
        {
            publish_rate_control = 0;

            message = new RosMessageTypes.Geometry.Pose
            {
                // position = RosTrans.Ros2Unity(target.transform.position),
                // orientation = RosTrans.Ros2Unity(target.transform.rotation)
                position = Unity2Msg(target.transform.position),
                orientation = Unity2Msg(target.transform.rotation)
            };
            // publish the message
            rosConnector.Send(Topic, message);
        }
        else 
        {
            publish_rate_control += update_time_step;
        }
    }

    private Point Unity2Msg(Vector3 vector3)
    {
        return new Point(vector3.z, -vector3.x, vector3.y);
    }

    private RosQuaternion Unity2Msg(Quaternion quaternion)
    {
        return new RosQuaternion((float)-quaternion.z, (float)quaternion.x, (float)-quaternion.y, (float)quaternion.w);
    }
}
