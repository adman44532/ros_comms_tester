
# Developed by Adam Pigram
#
# This is an overall class for measuring Round-Trip-Time (RTT) for a series
# of tests which inherit from this base class.
#
#
# IMPLEMENTATION
#
# 1. A child node extends the base __init__ with publishers, subscribers and
#    test specific globals (e.g. Message Size) 
# 2. All nodes must implement a publish_message method, accounting for new globals
# 3. If the returning message is not of std_msgs.String() type, then 
#    extract_message_id need reimplementation to return an int()

from rclpy.node import Node
from time import perf_counter
import csv

class RTTBaseNode(Node):
    def __init__(self, node_name, log_file='rtt_log.csv', timeout=2.0):
        super().__init__(node_name)
        
        self.log_file = log_file
        self.log_data = []
        self.timeout = timeout
        
        self.timer = self.create_timer(1.0, self.publish_message)
        self.timeout_checker = self.create_timer(0.5, self.check_for_timeouts)
        
        self.msg_count = 0
        self.send_times = {}
        
        self.get_logger().info(f'{node_name} has been started.')
        
    def publish_message(self):
        raise NotImplementedError("Subclasses must implement this method.")
    
    def listener_callback(self, msg):
        receive_time = perf_counter()
        msg_id = self.extract_message_id(msg)
        
        if msg_id is None:
            return
        
        if msg_id in self.send_times:
            send_time, is_lost = self.send_times.pop(msg_id)
            rtt = receive_time - send_time
            status = 'late' if is_lost else 'received'
            self.log_data.append({'message_id': msg_id, 'send_time': send_time, 'receive_time': receive_time, 'rtt': rtt, 'status': status})
            self.get_logger().info(f'Received response for message {msg_id}. RTT: {rtt:.6f} seconds ({status})')

            # Save to file after receiving a response
            self.save_rtt_log()
            
    def check_for_timeouts(self):
        current_time = perf_counter()
        
        # If a message is lost, then listener_callback is not envoked, so for that message, we append it as lost
        for msg_id, (send_time, is_lost) in list(self.send_times.items()):  # Create a list to safely modify the dictionary
            if not is_lost and (current_time - send_time > self.timeout):
                # Mark as lost
                self.get_logger().warn(f'Message {msg_id} timed out. Assuming packet loss.')
                self.send_times[msg_id] = (send_time, True)  # Mark as lost
                self.log_data.append({'message_id': msg_id, 'send_time': send_time, 'receive_time': None, 'rtt': None, 'status': 'lost'})
                # Save to file after detecting a timeout
                self.save_rtt_log()
                
    def extract_message_id(self, msg):
        """Default implementation for extracting message ID from a standard string format."""
        # Assume the message is in the format 'Hello from ROS2 <msg_count>'
        received_data = msg.data.split(' ', 3)  # Only split up to the first 3 spaces to extract msg_id
        if len(received_data) < 4:  # Expecting at least 4 parts: ['Hello', 'from', 'ROS2', '<msg_count>']
            return None  # Handle unexpected message formats

        try:
            return int(received_data[3].split(' ')[0])  # Extract the message identifier (the first number after 'ROS2')
        except ValueError:
            return None  # Handle case where conversion to int fails
        
    # CAUTION: Overwrites on rerun
    def save_rtt_log(self):
        # Write the logged data to a CSV file
        with open(self.log_file, 'w', newline='') as csvfile:
            fieldnames = ['message_id', 'send_time', 'receive_time', 'rtt', 'status']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.log_data)
        self.get_logger().info(f'Saved RTT data to {self.log_file}')