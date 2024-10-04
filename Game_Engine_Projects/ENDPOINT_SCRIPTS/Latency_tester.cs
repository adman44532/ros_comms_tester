// FOR Unity TCP Connector System

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using TMPro; // Namespace for TextMeshPro
using System.Collections.Concurrent; // Required for ConcurrentQueue

public class ROS2PubSub : MonoBehaviour
{
    ROSConnection ros;
    public string pubTopicName = "/latency_test_response";
    public string subTopicName = "/latency_test_request";

    // Reference to the TextMeshPro text object in your scene
    public TextMeshPro messageCountText;

    // Thread-safe queue to store received messages
    private ConcurrentQueue<StringMsg> messageQueue = new ConcurrentQueue<StringMsg>();

    // Variable to keep track of message count
    private int messageCount = 0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Register the publisher
        ros.RegisterPublisher<StringMsg>(pubTopicName);

        // Register the subscriber
        ros.Subscribe<StringMsg>(subTopicName, ReceiveMessage);
    }

    void ReceiveMessage(StringMsg message)
    {
        // Enqueue the received message
        messageQueue.Enqueue(message);
    }

    void Update()
    {
        // Process all messages in the queue
        while (messageQueue.TryDequeue(out StringMsg message))
        {
            // Publish the message
            ros.Publish(pubTopicName, message);

            // Increment the message count
            messageCount++;

            // Log the received message
            Debug.Log("Received and published message: " + message.data);
        }

        // Update the TextMeshPro text
        if (messageCountText != null)
        {
            messageCountText.text = "Messages Received: " + messageCount;
        }
    }
}
