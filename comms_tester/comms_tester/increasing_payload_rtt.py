# File: ros2_latency_test/increasing_payload_rtt.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import perf_counter
import csv


class IncreasingPayloadRTT(Node):
    def __init__(self):
        super().__init__('increasing_payload_rtt')

        # Parameters for logging
        self.log_file = 'increasing_payload_rtt_log.csv'
        self.log_data = []
        self.timeout = 2.0  # Timeout in seconds for detecting packet loss

        # Publisher and Subscriber setup
        self.publisher_ = self.create_publisher(String, 'latency_test_request', 10)
        self.subscriber_ = self.create_subscription(String, 'latency_test_response', self.listener_callback, 10)

        # Timer for sending messages and checking for timeouts
        self.timer = self.create_timer(1.0, self.publish_message)
        self.timeout_checker = self.create_timer(self.timeout, self.check_for_timeouts)  # Check timeouts every 0.5 seconds

        # Message count and time tracking
        self.msg_count = 0
        self.send_times = {}

        self.initial_payload_size = 1024  # Start with 1 KB
        self.payload_increment = 1024  # Increase payload size by 1 KB with each message
        self.current_payload_size = self.initial_payload_size

        self.get_logger().info('Increasing Payload RTT Node has been started.')

    def publish_message(self):
        # Construct a message with the current payload size
        large_payload = 'X' * self.current_payload_size
        msg = String()
        msg.data = f'Hello from ROS2 {self.msg_count} ' + large_payload
        send_time = perf_counter()  # Use perf_counter for high-resolution timing

        # Track message send time
        self.send_times[self.msg_count] = (send_time, False)  # Store time and a flag indicating if it's already marked as lost
        self.publisher_.publish(msg)

        self.get_logger().info(f'Published message {self.msg_count} with payload size {self.current_payload_size} bytes at {send_time:.6f}')
        self.msg_count += 1

        # Increase the payload size for the next message
        self.current_payload_size += self.payload_increment

    def listener_callback(self, msg):
        receive_time = perf_counter()  # Use perf_counter for high-resolution timing
        received_data = msg.data.split(' ', 3)  # Only split up to the first 3 spaces to extract msg_id correctly
        if len(received_data) < 4:  # Expecting at least 4 parts: ['Hello', 'from', 'ROS2', '<msg_count>...']
            return  # Handle unexpected message formats

        try:
            msg_id = int(received_data[3].split(' ')[0])  # Extract the message identifier (the first number after 'ROS2')
        except ValueError:
            return  # Handle case where conversion to int fails

        if msg_id in self.send_times:
            send_time, is_lost = self.send_times.pop(msg_id)  # Retrieve and remove the entry

            rtt = receive_time - send_time
            if is_lost:
                status = 'late'  # Packet arrived after timeout
                self.get_logger().info(f'Received late response for message {msg_id}. Late RTT: {rtt:.6f} seconds')
            else:
                status = 'received'
                self.get_logger().info(f'Received response for message {msg_id}. RTT: {rtt:.6f} seconds')

            self.log_data.append({'message_id': msg_id, 'send_time': send_time, 'receive_time': receive_time, 'rtt': rtt, 'status': status, 'payload_size': self.current_payload_size - self.payload_increment})

            # Save to file
            self.save_rtt_log()

    def check_for_timeouts(self):
        current_time = perf_counter()
        # Check for any messages that have timed out
        for msg_id, (send_time, is_lost) in list(self.send_times.items()):  # Create a list to safely modify the dictionary
            if not is_lost and (current_time - send_time > self.timeout):
                # Mark as lost
                self.get_logger().warn(f'Message {msg_id} timed out. Assuming packet loss.')
                self.send_times[msg_id] = (send_time, True)  # Mark as lost
                self.log_data.append({'message_id': msg_id, 'send_time': send_time, 'receive_time': None, 'rtt': None, 'status': 'lost', 'payload_size': self.current_payload_size - self.payload_increment})
                # Save to file
                self.save_rtt_log()

    def save_rtt_log(self):
        # Write the logged data to a CSV file
        with open(self.log_file, 'w', newline='') as csvfile:
            fieldnames = ['message_id', 'send_time', 'receive_time', 'rtt', 'status', 'payload_size']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.log_data)
        self.get_logger().info(f'Saved RTT data to {self.log_file}')


def main(args=None):
    rclpy.init(args=args)
    increasing_payload_rtt = IncreasingPayloadRTT()

    try:
        rclpy.spin(increasing_payload_rtt)
    except KeyboardInterrupt:
        increasing_payload_rtt.get_logger().info('Shutting down Increasing Payload RTT Node...')
    finally:
        increasing_payload_rtt.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
