# File: ros2_latency_test/large_payload_rtt.py

# Developed by Adam Pigram
#
# Test 2: Using a larger payload size, this script will use a 50KB string payload size
# All logic mirrors the Test 1 Simple string

import rclpy
from std_msgs.msg import String
from time import perf_counter
from comms_tester.RTTBaseNode import RTTBaseNode

class LargePayloadRTT(RTTBaseNode):
    def __init__(self):
        super().__init__(node_name='large_payload_rtt', log_file='large_payload_rtt_log.csv', timeout=2.0)
        
        self.publisher = self.create_publisher(String, 'latency_test_request')
        self.subscriber_ = self.create_subscription(String, 'latency_test_response', self.listener_callback, 10)
        
        self.large_payload = 'X' * 1024 * 50 # Roughly 50KB Payload
        
    def publish_message(self):
        msg = String()
        msg.data = f'Hello from ROS2 {self.msg_count} ' + self.large_payload
        send_time = perf_counter()
        self.publisher_.publish(msg)
        self.send_times[self.msg_count] = (send_time, False) # dict send_time, is_lost
        
        self.get_logger().info(f'Publisher message {self.msg_count} at {send_time}')
        self.msg_count += 1

def main(args=None):
    rclpy.init(args=args)
    large_payload_rtt = LargePayloadRTT()

    try:
        rclpy.spin(large_payload_rtt)
    except KeyboardInterrupt:
        large_payload_rtt.get_logger().info('Shutting down Large Payload RTT Node...')
    finally:
        large_payload_rtt.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
