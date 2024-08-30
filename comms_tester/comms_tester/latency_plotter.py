# File: ros2_latency_test/latency_plotter.py

# Developed by Adam Pigram
#
# This should serve as a blanket interpreter for all the latency tests.
# It outputs important values like average, std, var
# It also outputs a diagram, noting packets lost
#
# Script will assume that the csv will be present in the cwd

import csv
import os
import matplotlib.pyplot as plt
import numpy as np


def read_csv_data(csv_file_path):
    """Reads RTT data from the CSV file and returns it as a list of dictionaries."""
    data = []
    if not os.path.exists(csv_file_path):
        print(f"Error: The file '{csv_file_path}' does not exist.")
        return data

    with open(csv_file_path, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            # Convert numeric fields to float where applicable
            try:
                row['message_id'] = int(row['message_id'])
                row['send_time'] = float(row['send_time'])
                row['receive_time'] = float(row['receive_time']) if row['receive_time'] else None
                row['rtt'] = float(row['rtt']) if row['rtt'] else None
                data.append(row)
            except ValueError:
                print(f"Error: Failed to convert data row: {row}")  # Most likely a packet loss
                continue
    return data


def display_data(data):
    """Displays all relevant data points and statistics in the terminal, including packet loss information."""
    total_packets = len(data)
    received_data = [entry for entry in data if entry['rtt'] is not None]
    lost_data = [entry for entry in data if entry['rtt'] is None]

    if total_packets == 0:
        print("No data available to display.")
        return

    # Check if there is any received data
    if not received_data:
        print("No valid RTT data to display.")
        return

    rtts = [entry['rtt'] for entry in received_data]

    # Compute statistics
    avg_rtt = np.mean(rtts)
    median_rtt = np.median(rtts)
    min_rtt = np.min(rtts)
    max_rtt = np.max(rtts)
    range_rtt = max_rtt - min_rtt
    std_dev_rtt = np.std(rtts)
    variance_rtt = np.var(rtts)

    # Compute packet loss
    lost_packets = len(lost_data)
    packet_loss_percentage = (lost_packets / total_packets) * 100

    # Display statistics
    print("\nRTT Statistics:")
    print(f"Total Packets Sent: {total_packets}")
    print(f"Packets Received: {len(received_data)}")
    print(f"Packets Lost: {lost_packets}")
    print(f"Packet Loss Percentage: {packet_loss_percentage:.2f}%")
    print(f"Average RTT: {avg_rtt:.8f} s")
    print(f"Median RTT: {median_rtt:.8f} s")
    print(f"RTT Range: {min_rtt:.8f} - {max_rtt:.8f} s = {range_rtt:.8f} s")
    print(f"Standard Deviation of RTT: {std_dev_rtt:.8f} s")
    print(f"Variance of RTT: {variance_rtt:.8f} s")


def plot_rtt(data):
    """Plots the RTT graph using matplotlib and marks packet loss."""
    received_data = [entry for entry in data if entry['rtt'] is not None]
    lost_data = [entry for entry in data if entry['rtt'] is None]

    if not received_data:
        print("No valid RTT data to plot.")
        return

    # Data for received packets
    message_ids = [entry['message_id'] for entry in received_data]
    rtts = [entry['rtt'] for entry in received_data]

    # Data for lost packets
    lost_message_ids = [entry['message_id'] for entry in lost_data]

    plt.figure(figsize=(10, 5))

    # Plot RTTs for received packets
    plt.plot(message_ids, rtts, marker='o', linestyle='-', color='b', label='Received RTT')

    # Plot lost packets as red X marks
    plt.scatter(lost_message_ids, [0] * len(lost_message_ids), color='r', marker='x', s=100, label='Lost Packets')

    plt.xlabel('Message ID')
    plt.ylabel('RTT (seconds)')
    plt.title('Round-Trip Time (RTT) Analysis with Packet Loss')
    plt.grid(True)
    plt.legend()
    plt.show()


def main():
    csv_file_path = 'data/simple_string_rtt_log.csv'  # Modify this path if the CSV is in a different location

    # Read data from CSV
    data = read_csv_data(csv_file_path)

    if data:
        # Display data points and statistics in the terminal
        display_data(data)

        # Plot the RTT graph
        plot_rtt(data)
    else:
        print("No data available for plotting.")


if __name__ == '__main__':
    main()
