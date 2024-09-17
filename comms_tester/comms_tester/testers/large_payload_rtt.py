# Developed by Adam Pigram
#
# This script tests the latency of communication by recording round-trip times (RTT)
# between a published message and a received response. The RTTs are saved to a CSV file
# in a data/ folder where the script is run.
#
# The code uses two topics to send and receive messages and sends a large string payload.

import rclpy
from std_msgs.msg import String
from time import perf_counter
from RTTBaseNode import RTTBaseNode  # Adjust the import path based on your project structure
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class LargePayloadRTT(RTTBaseNode):
    def __init__(self):
        super().__init__(
            node_name="large_payload_rtt",
            log_file="large_payload_rtt_log_",
            message_interval=0.25,
            message_limit=1000,
        )

        self.publisher_ = self.create_publisher(String, "latency_test_request", 10)
        self.subscriber_ = self.create_subscription(
            String, "latency_test_response", self.listener_callback, qos_profile=qos_profile
        )

        self.large_payload = "X" * 1024 * 100  # Roughly 100KB Payload

    def publish_message(self):
        if self.message_limit_check():
            return
        msg = String()
        msg.data = f"Hello from ROS2 {self.msg_count} " + self.large_payload
        send_time = perf_counter()
        self.publisher_.publish(msg)
        self.send_times[self.msg_count] = (send_time, False)  # Track send time and initial receive status

        self.get_logger().info(f"Published message {self.msg_count} at {send_time:.6f}")
        self.msg_count += 1


def main(args=None):
    rclpy.init(args=args)
    large_payload_rtt = LargePayloadRTT()

    try:
        rclpy.spin(large_payload_rtt)
    except KeyboardInterrupt:
        large_payload_rtt.get_logger().info("Shutting down Large Payload RTT Node...")
    finally:
        large_payload_rtt.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
