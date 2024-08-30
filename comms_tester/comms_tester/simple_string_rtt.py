# File: ros2_latency_test/simple_string_rtt.py

# Developed by Adam Pigram
# 
# This will test the latency of communication via recording round trip times (RTT)
# between a published message and received response. Saving the RTT to a csv file
# at the location the script is run with.

# The code uses two topics to send and receive and sends a basic string message

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import perf_counter
import csv

class SimpleStringRTT(Node):
    def __init__(self):
        super().__init__('simple_string_rtt')
        
        # Parameters for logging
        self.log_file = 'simple_string_rtt_log.csv'
        self.log_data = []
        self.timeout = 2.0  # Timeout in seconds for detecting packet loss

        # Publisher and Subscriber setup
        self.publisher_ = self.create_publisher(String, 'latency_test_request', 10)
        self.subscriber_ = self.create_subscription(String, 'latency_test_response', self.listener_callback, 10)
        
        # Timer for sending messages and checking for timeouts
        self.timer = self.create_timer(1.0, self.publish_message)
        self.timeout_checker = self.create_timer(0.5, self.check_for_timeouts)  # Check timeouts every 0.5 seconds
        
        # Message count and time tracking
        self.msg_count = 0
        self.send_times = {}
        
        self.get_logger().info('Simple String RTT Node has been started.')

    def publish_message(self):
        msg = String()
        msg.data = f'Hello from ROS2 {self.msg_count}'
        send_time = perf_counter()  # Use perf_counter for high-resolution timing

        # Track message send time
        self.send_times[self.msg_count] = send_time
        self.publisher_.publish(msg)

        self.get_logger().info(f'Published message {self.msg_count} at {send_time:.6f}')
        self.msg_count += 1

    def listener_callback(self, msg):
        receive_time = perf_counter()  # High-resolution timing for RTT
        received_data = msg.data.split(' ')  # Split the string by spaces
        if len(received_data) != 4:  # Expecting 4 parts: ['Hello', 'from', 'ROS2', '<msg_count>']
            return  # Handle unexpected message formats

        try:
            msg_id = int(received_data[3])  # Extract the message identifier (the last part)
        except ValueError:
            return  # Handle case where conversion to int fails

        if msg_id in self.send_times:
            send_time = self.send_times.pop(msg_id)  # Retrieve and remove the send time
            rtt = receive_time - send_time
            self.log_data.append({'messsage_id': msg_id, 'send_time': send_time, 'receive_time': receive_time, 'rtt': rtt})
            self.get_logger().info(f'Received response for message {msg_id}. RTT: {rtt:.6f} seconds')

            self.save_rtt_log()

    def check_for_timeouts(self):
        current_time = perf_counter()
        # Check for any messages that have timed out
        for msg_id, send_time in list(self.send_times.items()):  # Create a list to safely modify the dictionary
            if current_time - send_time > self.timeout:
                # Mark as lost and remove from tracking
                self.get_logger().warn(f'Message {msg_id} timed out. Assuming packet loss.')
                self.log_data.append({'message_id': msg_id, 'send_time': send_time, 'receive_time': None, 'rtt': None, 'status': 'lost'})
                self.send_times.pop(msg_id)

                # Save to file
                self.save_rtt_log()

    def save_rtt_log(self):
        # Write the logged data to a CSV file
        with open(self.log_file, 'w', newline='') as csvfile:
            fieldnames = ['message_id', 'send_time', 'receive_time', 'rtt', 'status']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.log_data)
        self.get_logger().info(f'Saved RTT data to {self.log_file}')


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
