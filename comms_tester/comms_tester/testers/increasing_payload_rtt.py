# Developed by Adam Pigram
#
# This script tests the latency of communication by recording round-trip times (RTT)
# between a published message and a received response. The RTTs are saved to a CSV file
# in a data/ folder where the script is run.
#
# The code uses two topics to send and receive messages.
# The string payload will increase incrementally with each message.

import rclpy
from std_msgs.msg import String
from time import perf_counter
from comms_tester.RTTBaseNode import RTTBaseNode  # Adjust the import path based on your project structure
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class IncreasingPayloadRTT(RTTBaseNode):
    def __init__(self):
        super().__init__(
            node_name="increasing_payload_rtt",
            log_file="increasing_payload_rtt_log_",
            message_interval=0.25,
            message_limit=1000,
        )

        self.publisher_ = self.create_publisher(String, "latency_test_request", 10)
        self.subscriber_ = self.create_subscription(
            String, "latency_test_response", self.listener_callback, qos_profile=qos_profile
        )

        self.initial_payload_size = 1  # Starting payload size in bytes
        self.payload_increment = 1024  # Increase payload size by 1 KB with each message
        self.current_payload_size = self.initial_payload_size

    def publish_message(self):
        if self.message_limit_check():
            return
        large_payload = "X" * self.current_payload_size
        msg = String()
        msg.data = f"Hello from ROS2 {self.msg_count} " + large_payload
        send_time = perf_counter()  # Use perf_counter for high-resolution timing

        # Track message send time and initial received status (False)
        self.send_times[self.msg_count] = (send_time, False)
        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Published message {self.msg_count} with payload size {self.current_payload_size} bytes at {send_time:.6f}"
        )
        self.msg_count += 1

        # Increase the payload size for the next message
        self.current_payload_size += self.payload_increment


def main(args=None):
    rclpy.init(args=args)
    increasing_payload_rtt = IncreasingPayloadRTT()

    try:
        rclpy.spin(increasing_payload_rtt)
    except KeyboardInterrupt:
        increasing_payload_rtt.get_logger().info(
            "Shutting down Increasing Payload RTT Node..."
        )
    finally:
        increasing_payload_rtt.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
