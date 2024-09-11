# Developed by Adam Pigram
#
# This is an analysis tool to calculate a variety of statistics based on the RTT tests.
#
# File: comms_tester/latency_plotter.py

import csv
import os
import matplotlib.pyplot as plt
import numpy as np
import logging
from datetime import datetime, timedelta  # Import for date and time operations

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def read_csv_data(csv_file_path):
    print("Reading CSV Data")
    """Reads RTT data from the CSV file, skipping system information lines, and returns it as a list of dictionaries."""
    data = []
    system_info = []
    if not os.path.exists(csv_file_path):
        logger.error(f"The file '{csv_file_path}' does not exist.")
        return system_info, data

    # Open the file and filter out the lines starting with '#'
    with open(csv_file_path, mode="r") as file:
        lines = file.readlines()

    # Separate system information and data lines
    for line in lines:
        if line.startswith("#"):
            system_info.append(line)
        else:
            break

    data_lines = [line for line in lines if not line.startswith("#")]

    # Use csv.DictReader to parse the cleaned data lines
    reader = csv.DictReader(data_lines)

    for row in reader:
        try:
            # Convert numeric fields to float where applicable
            row["message_id"] = int(row["message_id"])
            row["send_time"] = float(row["send_time"])
            row["receive_time"] = (
                float(row["receive_time"]) if row["receive_time"] else None
            )
            row["rtt"] = float(row["rtt"]) if row["rtt"] else None
            data.append(row)
        except ValueError:
            logger.error(
                f"Failed to convert data row: {row}"
            )  # Most likely a packet loss
            continue

    return system_info, data


def compute_test_duration(system_info, data):
    """Computes the test duration based on the start time and last data point."""
    # Extract start time from system info
    start_time_line = next((line for line in system_info if "Start Time" in line), None)
    if not start_time_line:
        return "0 seconds"

    start_time_str = start_time_line.split(":", 1)[1].strip()
    start_dt = datetime.strptime(start_time_str, "%Y-%m-%d %H:%M:%S")

    # Determine the end time
    if not data:
        return "0 seconds"

    # Convert the end time from perf_counter (seconds since start) to datetime
    last_entry = data[-1]
    end_time = (
        last_entry["receive_time"]
        if last_entry["receive_time"]
        else last_entry["send_time"]
    )
    end_dt = start_dt + timedelta(
        seconds=end_time - data[0]["send_time"]
    )  # Calculate relative to start

    # Compute duration
    duration = end_dt - start_dt
    duration_str = str(duration).split(".")[0]
    return str(duration_str)


def display_data(data):
    print("Computing statistics")
    """Displays all relevant data points and statistics in the terminal, including packet loss information."""
    total_packets = len(data)
    received_data = [entry for entry in data if entry["rtt"] is not None]
    lost_data = [entry for entry in data if entry["rtt"] is None]

    if total_packets == 0:
        logger.info("No data available to display.")
        return {}

    # Check if there is any received data
    if not received_data:
        logger.info("No valid RTT data to display.")
        return {}

    rtts = [entry["rtt"] for entry in received_data]

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

    # Return statistics as a dictionary
    return {
        "Total Packets Sent": total_packets,
        "Packets Received": len(received_data),
        "Packets Lost": lost_packets,
        "Packet Loss Percentage": f"{packet_loss_percentage:.2f}%",
        "Average RTT": f"{avg_rtt:.8f} s",
        "Median RTT": f"{median_rtt:.8f} s",
        "RTT Range": f"{range_rtt:.8f} s",
        "Standard Deviation of RTT": f"{std_dev_rtt:.8f} s",
        "Variance of RTT": f"{variance_rtt:.8f} s",
    }


def plot_rtt(data, output_path):
    print("Creating plots")
    """Plots the RTT graph using matplotlib, marks packet loss, and saves the plot as a PNG file."""
    received_data = [entry for entry in data if entry["rtt"] is not None]
    lost_data = [entry for entry in data if entry["rtt"] is None]

    if not received_data:
        logger.info("No valid RTT data to plot.")
        return

    # Data for received packets
    message_ids = [entry["message_id"] for entry in received_data]
    rtts = [entry["rtt"] for entry in received_data]

    # Data for lost packets
    lost_message_ids = [entry["message_id"] for entry in lost_data]

    plt.figure(figsize=(10, 5))

    # Plot RTTs for received packets
    plt.plot(
        message_ids, rtts, marker="o", linestyle="-", color="b", label="Received RTT"
    )

    # Plot lost packets as red X marks
    plt.scatter(
        lost_message_ids,
        [0] * len(lost_message_ids),
        color="r",
        marker="x",
        s=100,
        label="Lost Packets",
    )

    # Calculate line of best fit
    if (
        len(message_ids) > 1
    ):  # Ensure there is enough data to calculate a line of best fit
        coeffs = np.polyfit(message_ids, rtts, 1)  # 1st degree polynomial (linear fit)
        best_fit_line = np.poly1d(coeffs)

        # Plot the line of best fit with improved visibility
        plt.plot(
            message_ids,
            best_fit_line(message_ids),
            color="red",  # Change color to red for better visibility
            linestyle="-",  # Use a solid line for clarity
            linewidth=4,  # Increase line width to make it thicker
            label="Best Fit Line",
        )

    plt.xlabel("Message ID")
    plt.ylabel("RTT (seconds)")
    plt.title("Round-Trip Time (RTT) Analysis with Packet Loss")
    plt.grid(True)
    plt.legend()

    # Save the plot as a PNG file
    plt.savefig(output_path)
    logger.info(f"Plot saved as {output_path}")
    plt.close()


def write_statistics_to_csv(csv_file_path, statistics, test_duration):
    print("Appending data to csv")
    """Appends computed statistics in tabbed format to the CSV file."""
    with open(csv_file_path, "r", encoding="utf-8") as csvfile:
        lines = csvfile.readlines()

    # Find the location to insert computed statistics
    statistics_index = lines.index("# Computed Statistics\n") + 1

    # Update the statistics in tabbed format
    lines[statistics_index : statistics_index + 2] = [
        f"# Total Packets Sent       : {statistics['Total Packets Sent']}\n",
        f"# Packets Received         : {statistics['Packets Received']}\n",
        f"# Packets Lost             : {statistics['Packets Lost']}\n",
        f"# Packet Loss Percentage   : {statistics['Packet Loss Percentage']}\n",
        f"# Average RTT              : {statistics['Average RTT']}\n",
        f"# Median RTT               : {statistics['Median RTT']}\n",
        f"# RTT Range                : {statistics['RTT Range']}\n",
        f"# Standard Deviation of RTT: {statistics['Standard Deviation of RTT']}\n",
        f"# Variance of RTT          : {statistics['Variance of RTT']}\n",
        f"# Test Duration            : {test_duration}\n",
        "#\n",
    ]

    # Write back the updated lines to the CSV file
    with open(csv_file_path, "w", encoding="utf-8") as csvfile:
        csvfile.writelines(lines)


def main():
    print("Starting latency plotter")
    # Define the directory containing the CSV files
    data_folder = "data"

    # Check if the data folder exists
    if not os.path.exists(data_folder):
        logger.error(f"The data folder '{data_folder}' does not exist.")
        return

    # Get all CSV files in the data folder
    csv_files = [f for f in os.listdir(data_folder) if f.endswith(".csv")]

    if not csv_files:
        logger.info("No CSV files found in the data folder.")
        return

    for csv_file in csv_files:
        csv_file_path = os.path.join(data_folder, csv_file)

        # Read data from CSV
        system_info, data = read_csv_data(csv_file_path)

        if data:
            # Compute test duration
            test_duration = compute_test_duration(system_info, data)

            # Display data points and statistics in the terminal
            statistics = display_data(data)

            # Append computed statistics to CSV
            write_statistics_to_csv(csv_file_path, statistics, test_duration)

            # Plot the RTT graph and save as PNG
            output_path = os.path.join(
                data_folder, f"{os.path.splitext(csv_file)[0]}.png"
            )
            plot_rtt(data, output_path)
        else:
            logger.info(f"No data available for plotting from file '{csv_file}'.")
    print("Latency Plotter complete")


if __name__ == "__main__":
    main()
