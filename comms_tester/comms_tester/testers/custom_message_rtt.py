# File: vr_comms/custom_message_rtt.py

import rclpy
from comms_interfaces.msg import CustomMessage  # Import the custom message type
from time import perf_counter
from comms_tester.RTTBaseNode import RTTBaseNode

class CustomMessageRTT(RTTBaseNode):
    def __init__(self):
        super().__init__('custom_message_rtt', log_file='custom_message_rtt_log.csv')

        # Publisher and Subscriber setup
        self.publisher_ = self.create_publisher(CustomMessage, 'latency_test_request', 10)
        self.subscriber_ = self.create_subscription(CustomMessage, 'latency_test_response', self.listener_callback, 10)

    def publish_message(self):
        # Construct a custom message
        msg = CustomMessage()
        msg.message_id = self.msg_count
        msg.content = f'Hello from ROS2 {self.msg_count}'
        msg.timestamp = perf_counter()  # Use perf_counter for high-resolution timing

        # Track message send time
        self.send_times[self.msg_count] = (msg.timestamp, False)
        self.publisher_.publish(msg)

        self.get_logger().info(f'Published custom message {self.msg_count} at {msg.timestamp:.6f}')
        self.msg_count += 1

    def extract_message_id(self, msg):
        """Extract message ID from a custom message format."""
        return msg.message_id  # Directly return the message_id from the custom message

def main(args=None):
    rclpy.init(args=args)
    custom_message_rtt = CustomMessageRTT()

    try:
        rclpy.spin(custom_message_rtt)
    except KeyboardInterrupt:
        custom_message_rtt.get_logger().info('Shutting down Custom Message RTT Node...')
    finally:
        custom_message_rtt.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
