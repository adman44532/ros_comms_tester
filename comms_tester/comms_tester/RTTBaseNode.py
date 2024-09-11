
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

import csv
import os
import rclpy
from rclpy.node import Node
from time import perf_counter
import platform
from datetime import datetime  # Import for date and time

class RTTBaseNode(Node):
    def __init__(self, node_name, log_file='rtt_log', timeout=2.0, message_limit=0):
        super().__init__(node_name)
        self.start_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        full_log_file = log_file + self.start_time + ".csv"
        
        # Ensure the file is saved in the data folder
        self.log_file = os.path.join('data', full_log_file)
        self.log_data = []
        self.timeout = timeout

        # Get the start date and time
        
        # Create the data directory if it does not exist
        os.makedirs('data', exist_ok=True)

        # Write initial system information to the log file
        self.write_system_info()  
        
        self.timer = self.create_timer(1.0, self.publish_message)
        self.timeout_checker = self.create_timer(0.5, self.check_for_timeouts)
        
        self.message_limit = message_limit
        self.msg_count = 0
        self.send_times = {}
        
        self.get_logger().info(f'{node_name} has been started.')

    def write_system_info(self):
        """Writes system information to the top of the CSV log file in tabbed format."""
        system_info = {
            'OS': platform.system(),
            'OS Version': platform.version(),
            'Architecture': platform.machine(),
            'Python Version': platform.python_version(),
            'Processor': platform.processor(),
            'Node Name': self.get_name(),
            'Start Time': self.start_time  # Include start time
        }

        # Open CSV in write mode to add the header
        with open(self.log_file, 'w', newline='', encoding='utf-8') as csvfile:
            # Write system information in tabbed format
            csvfile.write('# System Information\n')
            for key, value in system_info.items():
                csvfile.write(f'# {key:<23}: {value}\n')
            csvfile.write('#\n')

            # Placeholder for computed statistics (to be filled by the plotter)
            csvfile.write('# Computed Statistics\n')
            csvfile.write('# Run latency_plotter.py to compute statistics\n')
            csvfile.write('#\n')

            # Write column headers required by latency_plotter.py
            fieldnames = ['message_id', 'send_time', 'receive_time', 'rtt', 'status']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def publish_message(self):
        raise NotImplementedError("Subclasses must implement this method.")
    
    def message_limit_check(self):
        if self.message_limit > 0 and self.msg_count >= self.message_limit:
            self.get_logger().info('Reached message limit, stopping message publishing.')
            self.timer.cancel()  # Stop the timer to stop publishing
            self.destroy_node()
            return True
        return False
    
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
        
        # If a message is lost, then listener_callback is not invoked, so for that message, we append it as lost
        for msg_id, (send_time, is_lost) in list(self.send_times.items()):
            if not is_lost and (current_time - send_time > self.timeout):
                # Mark as lost
                self.get_logger().warn(f'Message {msg_id} timed out. Assuming packet loss.')
                self.send_times[msg_id] = (send_time, True)
                self.log_data.append({'message_id': msg_id, 'send_time': send_time, 'receive_time': None, 'rtt': None, 'status': 'lost'})
                # Save to file after detecting a timeout
                self.save_rtt_log()

    def extract_message_id(self, msg):
        """Default implementation for extracting message ID from a standard string format."""
        received_data = msg.data.split(' ', 3)
        if len(received_data) < 4:
            return None
        try:
            return int(received_data[3].split(' ')[0])
        except ValueError:
            return None

    def save_rtt_log(self):
        """Saves RTT log data to a CSV file."""
        # Write the logged data to a CSV file
        with open(self.log_file, 'a', newline='', encoding='utf-8') as csvfile:
            fieldnames = ['message_id', 'send_time', 'receive_time', 'rtt', 'status']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerows(self.log_data)
            self.log_data = []  # Clear logged data after writing
        self.get_logger().info(f'Saved RTT data to {self.log_file}')