# Developed by Adam Pigram
#
# This script tests the latency of communication by recording round-trip times (RTT)
# between a published message and a received response. The RTTs are saved to a CSV file
# in a data/ folder where the script is run.
#
# The code uses two topics to send and receive messages.
# It sends a basic string message.

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

class SimpleStringRTT(RTTBaseNode):
    def __init__(self):
        super().__init__(
            node_name="simple_string_rtt",
            log_file="simple_string_rtt_log_",
            message_interval=0.25,
            message_limit=1000,
        )

        self.publisher_ = self.create_publisher(String, "latency_test_request", 10)
        self.subscriber_ = self.create_subscription(
            String, "latency_test_response", self.listener_callback, qos_profile=qos_profile
        )

    # OVERRIDE
    def publish_message(self):
        if self.message_limit_check():
            return
        msg = String()
        msg.data = f"Hello from ROS2 {self.msg_count}"
        send_time = perf_counter()
        self.publisher_.publish(msg)
        self.send_times[self.msg_count] = (send_time, False)  # Indicate not yet received

        self.get_logger().info(f"Published message {self.msg_count} at {send_time}")
        self.msg_count += 1

def main(args=None):
    rclpy.init(args=args)
    simple_string_rtt = SimpleStringRTT()

    try:
        rclpy.spin(simple_string_rtt)
    except KeyboardInterrupt:
        simple_string_rtt.get_logger().info("Shutting down Simple String RTT Node...")
    finally:
        simple_string_rtt.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
