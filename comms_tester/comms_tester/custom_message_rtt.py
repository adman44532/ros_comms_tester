# File: ros2_latency_test/custom_message_rtt.py

import rclpy
from rclpy.node import Node
from comms_interface.msg import CustomMessage  # Import the custom message type
from time import perf_counter
import csv


class CustomMessageRTT(Node):
    def __init__(self):
        super().__init__('custom_message_rtt')

        # Parameters for logging
        self.log_file = 'custom_message_rtt_log.csv'
        self.log_data = []
        self.timeout = 2.0  # Timeout in seconds for detecting packet loss

        # Publisher and Subscriber setup
        self.publisher_ = self.create_publisher(CustomMessage, 'latency_test_request', 10)
        self.subscriber_ = self.create_subscription(CustomMessage, 'latency_test_response', self.listener_callback, 10)

        # Timer for sending messages and checking for timeouts
        self.timer = self.create_timer(1.0, self.publish_message)
        self.timeout_checker = self.create_timer(0.5, self.check_for_timeouts)  # Check timeouts every 0.5 seconds

        # Message count and time tracking
        self.msg_count = 0
        self.send_times = {}

        self.get_logger().info('Custom Message RTT Node has been started.')

    def publish_message(self):
        # Construct a custom message
        msg = CustomMessage()
        msg.message_id = self.msg_count
        msg.content = f'Hello from ROS2 {self.msg_count}'
        msg.timestamp = perf_counter()  # Use perf_counter for high-resolution timing

        # Track message send time
        self.send_times[self.msg_count] = (msg.timestamp, False)  # Store time and a flag indicating if it's already marked as lost
        self.publisher_.publish(msg)

        self.get_logger().info(f'Published custom message {self.msg_count} at {msg.timestamp:.6f}')
        self.msg_count += 1

    def listener_callback(self, msg):
        receive_time = perf_counter()  # Use perf_counter for high-resolution timing
        msg_id = msg.message_id

        if msg_id in self.send_times:
            send_time, is_lost = self.send_times.pop(msg_id)  # Retrieve and remove the entry

            rtt = receive_time - send_time
            if is_lost:
                status = 'late'  # Packet arrived after timeout
                self.get_logger().info(f'Received late response for custom message {msg_id}. Late RTT: {rtt:.6f} seconds')
            else:
                status = 'received'
                self.get_logger().info(f'Received response for custom message {msg_id}. RTT: {rtt:.6f} seconds')

            self.log_data.append({'message_id': msg_id, 'send_time': send_time, 'receive_time': receive_time, 'rtt': rtt, 'status': status})

            # Save to file
            self.save_rtt_log()

    def check_for_timeouts(self):
        current_time = perf_counter()
        # Check for any messages that have timed out
        for msg_id, (send_time, is_lost) in list(self.send_times.items()):  # Create a list to safely modify the dictionary
            if not is_lost and (current_time - send_time > self.timeout):
                # Mark as lost
                self.get_logger().warn(f'Custom message {msg_id} timed out. Assuming packet loss.')
                self.send_times[msg_id] = (send_time, True)  # Mark as lost
                self.log_data.append({'message_id': msg_id, 'send_time': send_time, 'receive_time': None, 'rtt': None, 'status': 'lost'})
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
