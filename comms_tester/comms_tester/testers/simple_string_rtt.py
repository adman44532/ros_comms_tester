# File: ros2_latency_test/simple_string_rtt.py

# Developed by Adam Pigram
# 
# This will test the latency of communication via recording round trip times (RTT)
# between a published message and received response. Saving the RTT to a csv file
# at the location the script is run with.

# The code uses two topics to send and receive and sends a basic string message

import rclpy
from std_msgs.msg import String
from time import perf_counter
from comms_tester.RTTBaseNode import RTTBaseNode

class SimpleStringRTT(RTTBaseNode):
    def __init__(self):
        super().__init__(node_name='simple_string_rtt', log_file='simple_string_rtt_log.csv', timeout=2.0)
        
        self.publisher_ = self.create_publisher(String, 'latency_test_request', 10)
        self.subscriber_ = self.create_subscription(String, 'latency_test_response', self.listener_callback, 10)
        
    # OVERRIDE
    def publish_message(self):
        msg = String()
        msg.data = f'Hello from ROS2 {self.msg_count}'
        send_time = perf_counter()
        self.publisher_.publish(msg)
        self.send_times[self.msg_count] = (send_time, False) # dict send_time, is_lost
        
        self.get_logger().info(f'Publisher message {self.msg_count} at {send_time}')
        self.msg_count += 1


def main(args=None):
    rclpy.init(args=args)
    simple_string_rtt = SimpleStringRTT()

    try:
        rclpy.spin(simple_string_rtt)
    except KeyboardInterrupt:
        simple_string_rtt.get_logger().info('Shutting down Simple String RTT Node...')
    finally:
        simple_string_rtt.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
