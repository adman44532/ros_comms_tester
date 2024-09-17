# Developed by Adam Pigram
#
# This is an overall class for measuring Round-Trip-Time (RTT) for a series
# of tests which inherit from this base class.
#
# IMPLEMENTATION
#
# 1. A child node calls super().__init__(), then declares publishers, subscribers and
#    test specific globals (e.g. Message Size)
# 2. All nodes must implement a publish_message method, accounting for new globals
# 3. If the returning message is not of std_msgs.String() type, then
#    extract_message_id needs reimplementation to return an int()

import csv
import os
from rclpy.node import Node
from time import perf_counter
import platform
from datetime import datetime  # Import for date and time


class RTTBaseNode(Node):
    def __init__(
        self,
        node_name='rtt_node',
        log_file="rtt_log_",
        message_interval=0.5,
        message_limit=0,
    ):
        super().__init__(node_name)
        self.start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        full_log_file = log_file + self.start_time + ".csv"

        # Ensure the file is saved in the data folder
        self.log_file = os.path.join("data", full_log_file)
        self.log_data = []

        # Create the data directory if it does not exist
        os.makedirs("data", exist_ok=True)

        # Write initial system information to the log file
        self.write_system_info()

        self.timer = self.create_timer(message_interval, self.publish_message)

        self.message_limit = message_limit
        self.msg_count = 0
        self.send_times = {}  # Format: {message_id: (send_time, is_received)}

        self.get_logger().info(f"{node_name} has been started.")

    def write_system_info(self):
        """Writes system information to the top of the CSV log file in tabbed format."""
        system_info = {
            "OS": platform.system(),
            "OS Version": platform.version(),
            "Architecture": platform.machine(),
            "Python Version": platform.python_version(),
            "Processor": platform.processor(),
            "Node Name": self.get_name(),
            "Start Time": self.start_time,  # Include start time
        }

        # Open CSV in write mode to add the header
        with open(self.log_file, "w", newline="", encoding="utf-8") as csvfile:
            # Write system information in tabbed format
            csvfile.write("# System Information\n")
            for key, value in system_info.items():
                csvfile.write(f"# {key:<25}: {value}\n")
            csvfile.write("#\n")

            # Placeholder for computed statistics (to be filled by the plotter)
            csvfile.write("# Computed Statistics\n")
            csvfile.write("# Run latency_plotter.py to compute statistics\n")
            csvfile.write("#\n")

            # Write column headers required by latency_plotter.py
            fieldnames = ["message_id", "send_time", "receive_time", "rtt", "status"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def publish_message(self):
        raise NotImplementedError("Subclasses must implement this method.")

    def message_limit_check(self):
        if self.message_limit > 0 and self.msg_count >= self.message_limit:
            self.get_logger().info(
                "Reached message limit, stopping message publishing."
            )
            self.timer.cancel()  # Stop the timer to stop publishing

            # Give extra time for messages to be received
            self.get_logger().info("Waiting for remaining messages to arrive...")
            self.shutdown_timer = self.create_timer(5.0, self.shutdown)  # Wait 5 seconds
            return True
        return False

    def shutdown(self):
        self.get_logger().info("Shutting down node...")
        self.shutdown_timer.cancel()
        self.destroy_node()

    def listener_callback(self, msg):
        receive_time = perf_counter()
        msg_id = self.extract_message_id(msg)

        if msg_id is None:
            self.get_logger().warn("Received a message with no valid ID.")
            return

        if msg_id in self.send_times:
            send_time, is_received = self.send_times[msg_id]
            rtt = receive_time - send_time
            status = "received"
            self.log_data.append(
                {
                    "message_id": msg_id,
                    "send_time": send_time,
                    "receive_time": receive_time,
                    "rtt": rtt,
                    "status": status,
                }
            )
            # Mark the message as received
            self.send_times[msg_id] = (send_time, True)

            # Save to file after receiving a response
            self.save_rtt_log()
        else:
            self.get_logger().warn(f"Received unexpected message ID: {msg_id}")

    def extract_message_id(self, msg):
        """Optimized method to extract message ID from the beginning of the string."""
        # Assume the message format starts with "Hello from ROS2 {message_id} "
        try:
            # Find the position where the payload starts
            prefix = "Hello from ROS2 "
            if msg.data.startswith(prefix):
                # Extract the part just after the prefix up to the next space
                id_part = msg.data[len(prefix):].split(' ', 1)[0]
                return int(id_part)  # Convert the extracted ID to an integer
        except (ValueError, IndexError):
            self.get_logger().warn("Failed to extract message ID from data.")
            return None


    def save_rtt_log(self):
        """Saves RTT log data to a CSV file."""
        # Write the logged data to a CSV file
        with open(self.log_file, "a", newline="", encoding="utf-8") as csvfile:
            fieldnames = ["message_id", "send_time", "receive_time", "rtt", "status"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerows(self.log_data)
            self.log_data = []  # Clear logged data after writing
        # Uncomment below if you want to log each save action
        # self.get_logger().info(f"Saved RTT data to {self.log_file}")

    def check_for_unreceived_messages(self):
        """Checks for messages that were not received before shutdown and logs them as lost."""
        # Identify messages that were not received
        for msg_id, (send_time, is_received) in self.send_times.items():
            if not is_received:
                self.get_logger().warn(
                    f"Message {msg_id} was not received before shutdown. Marking as lost."
                )
                self.log_data.append(
                    {
                        "message_id": msg_id,
                        "send_time": send_time,
                        "receive_time": None,
                        "rtt": None,
                        "status": "lost",
                    }
                )
        # Save any remaining log data
        self.save_rtt_log()

    def destroy_node(self):
        """Overrides the default destroy_node to check for unreceived messages before shutting down."""
        # Check for unreceived messages before shutting down
        self.check_for_unreceived_messages()
        super().destroy_node()
