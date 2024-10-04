// For Unity2ROS2 Package

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ROS2
{

    public class ROS2_Latency_Responder : MonoBehaviour
    {
        // Start is called before the first frame update
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<std_msgs.msg.String> chatter_pub;
        private ISubscription<std_msgs.msg.String> chatter_sub;
        private int i;

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
        }

        void listener_callback(std_msgs.msg.String msg) {
            Debug.Log("Unity listener ping!");
            chatter_pub.Publish(msg);
        }

        void Update()
        {
            if (ros2Unity.Ok())
            {
                if (ros2Node == null)
                {
                    ros2Node = ros2Unity.CreateNode("ROS2UnityTalkerNode");
                    chatter_pub = ros2Node.CreatePublisher<std_msgs.msg.String>("latency_test_response");
                    chatter_sub = ros2Node.CreateSubscription<std_msgs.msg.String>(
                        "latency_test_request", msg => listener_callback(msg));
                }

                // i++;
                // std_msgs.msg.String msg = new std_msgs.msg.String();
                // msg.Data = "Unity ROS2 sending: hello " + i;
                // chatter_pub.Publish(msg);
            }
        }
    }
}