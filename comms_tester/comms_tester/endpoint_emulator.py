# File: ros2_latency_test/endpoint_emulator.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EndpointEmulator(Node):
    def __init__(self):
        super().__init__('endpoint_emulator')
        
        # Publisher to send responses
        self.publisher_ = self.create_publisher(String, 'latency_test_response', 10)
        
        # Subscriber to receive messages
        self.subscriber_ = self.create_subscription(String, 'latency_test_request', self.listener_callback, 10)
        
        self.get_logger().info('Endpoint Emulator Node has been started.')

    def listener_callback(self, msg):
        # Extract the received message
        self.get_logger().info(f'Received message')
        
        # Create a response message
        response_msg = String()
        response_msg.data = msg.data  # Echo the same message back

        # Publish the response message
        self.publisher_.publish(response_msg)
        self.get_logger().info(f'Published response')


def main(args=None):
    rclpy.init(args=args)
    endpoint_emulator = EndpointEmulator()

    try:
        rclpy.spin(endpoint_emulator)
    except KeyboardInterrupt:
        endpoint_emulator.get_logger().info('Shutting down Endpoint Emulator Node...')
    finally:
        endpoint_emulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
