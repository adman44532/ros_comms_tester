# File: comms_tester/simple_string_rtt.py

# Developed by Adam Pigram
# 
# This will test the latency of communication via recording round trip times (RTT)
# between a published message and received response. Saving the RTT to a csv file
# in a data/ folder where the script in run in.

# The code uses two topics to send and receive.
# Sends a basic string

import rclpy
from std_msgs.msg import String
from time import perf_counter
from comms_tester.RTTBaseNode import RTTBaseNode

class SimpleStringRTT(RTTBaseNode):
    def __init__(self):
        # Change Variables here
        super().__init__(node_name='simple_string_rtt', log_file='simple_string_rtt_log', timeout=2.0, message_limit=1000)
        
        self.publisher_ = self.create_publisher(String, 'latency_test_request', 10)
        self.subscriber_ = self.create_subscription(String, 'latency_test_response', self.listener_callback, 10)
        
    # OVERRIDE
    def publish_message(self):
        if self.message_limit_check():
            return
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
