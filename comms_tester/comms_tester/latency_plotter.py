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
from datetime import datetime, timedelta
import argparse  # Import for command-line argument parsing

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


def is_processed(system_info):
    """Checks if the CSV file has already been processed by looking for computed statistics."""
    for line in system_info:
        if line.startswith("# Run latency_plotter.py to compute statistics"):
            return False
    return True


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


def display_data(data, threshold_time):
    print("Computing statistics")
    """Displays all relevant data points and statistics in the terminal, including packet loss information."""
    total_packets = len(data)
    received_data = [entry for entry in data if entry["rtt"] is not None]
    lost_data = [entry for entry in data if entry["rtt"] is None]

    if total_packets == 0:
        logger.info("No data available to display.")
        return {}

    # Compute packet loss
    lost_packets = len(lost_data)
    packet_loss_percentage = (lost_packets / total_packets) * 100

    # Initialize statistics dictionary
    statistics = {
        "Total Packets Sent": total_packets,
        "Packets Received": len(received_data),
        "Packets Lost": lost_packets,
        "Packet Loss Percentage": f"{packet_loss_percentage:.2f}%",
        # Initialize RTT statistics with 'N/A'
        "Average RTT": "N/A",
        "Median RTT": "N/A",
        "RTT Range": "N/A",
        "Standard Deviation of RTT": "N/A",
        "Variance of RTT": "N/A",
        "Packets Under Threshold": 0,  # Corrected key name
        "Packets Over Threshold": 0,
    }

    # Check if there is any received data
    if not received_data:
        logger.info("No valid RTT data to display.")
        return statistics

    rtts = [entry["rtt"] for entry in received_data]

    # Compute RTT statistics
    avg_rtt = np.mean(rtts)
    median_rtt = np.median(rtts)
    min_rtt = np.min(rtts)
    max_rtt = np.max(rtts)
    range_rtt = max_rtt - min_rtt
    std_dev_rtt = np.std(rtts)
    variance_rtt = np.var(rtts)

    # Classify packets based on threshold
    packets_under_threshold = sum(1 for rtt in rtts if rtt <= threshold_time)
    packets_over_threshold = sum(1 for rtt in rtts if rtt > threshold_time)

    # Update statistics with computed RTT values and threshold counts
    statistics.update(
        {
            "Average RTT": f"{avg_rtt:.8f} s",
            "Median RTT": f"{median_rtt:.8f} s",
            "RTT Range": f"{range_rtt:.8f} s",
            "Standard Deviation of RTT": f"{std_dev_rtt:.8f} s",
            "Variance of RTT": f"{variance_rtt:.8f} s",
            "Packets Under Threshold": packets_under_threshold,  # Corrected key name
            "Packets Over Threshold": packets_over_threshold,
        }
    )

    # Display statistics
    logger.info(f"Total Packets Sent       : {statistics['Total Packets Sent']}")
    logger.info(f"Packets Received         : {statistics['Packets Received']}")
    logger.info(f"Packets Lost             : {statistics['Packets Lost']}")
    logger.info(f"Packet Loss Percentage   : {statistics['Packet Loss Percentage']}")
    logger.info(f"Average RTT              : {statistics['Average RTT']}")
    logger.info(f"Median RTT               : {statistics['Median RTT']}")
    logger.info(f"RTT Range                : {statistics['RTT Range']}")
    logger.info(f"Standard Deviation of RTT: {statistics['Standard Deviation of RTT']}")
    logger.info(f"Variance of RTT          : {statistics['Variance of RTT']}")
    logger.info(f"Packets Under Threshold  : {statistics['Packets Under Threshold']}")  # Corrected key name
    logger.info(f"Packets Over Threshold   : {statistics['Packets Over Threshold']}")

    return statistics



def plot_rtt(data, output_path, threshold_time):
    print("Creating plots")
    """Plots the RTT graph using matplotlib, marks packet loss and over-threshold packets, and saves the plot as a PNG file."""
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

    # Classify received packets based on threshold
    under_threshold_ids = [
        entry["message_id"] for entry in received_data if entry["rtt"] <= threshold_time
    ]
    under_threshold_rtts = [
        entry["rtt"] for entry in received_data if entry["rtt"] <= threshold_time
    ]

    over_threshold_ids = [
        entry["message_id"] for entry in received_data if entry["rtt"] > threshold_time
    ]
    over_threshold_rtts = [
        entry["rtt"] for entry in received_data if entry["rtt"] > threshold_time
    ]

    # Scatter plot for RTT analysis
    plt.figure(figsize=(12, 6))

    # Plot RTTs for packets under threshold
    plt.scatter(
        under_threshold_ids,
        under_threshold_rtts,
        marker="o",
        color="b",
        label="Meets Threshold",
    )

    # Plot RTTs for packets over threshold
    plt.scatter(
        over_threshold_ids,
        over_threshold_rtts,
        marker="o",
        color="orange",
        label="Over Threshold",
    )

    # Plot lost packets as red X marks at RTT = 0
    plt.scatter(
        lost_message_ids,
        [0] * len(lost_message_ids),
        color="r",
        marker="x",
        s=100,
        label="Lost Packets",
    )

    # Draw the threshold line
    plt.axhline(
        y=threshold_time,
        color="g",
        linestyle="--",
        label=f"Threshold RTT = {threshold_time:.4f}s",
    )

    # Calculate and plot line of best fit for all received RTTs
    if len(message_ids) > 1:  # Ensure there's enough data for a line of best fit
        coeffs = np.polyfit(message_ids, rtts, 1)  # 1st degree polynomial (linear fit)
        best_fit_line = np.poly1d(coeffs)

        # Plot the line of best fit
        plt.plot(
            message_ids,
            best_fit_line(message_ids),
            color="red",
            linestyle="-",
            linewidth=2,
            label="Line of Best Fit",
        )

    plt.xlabel("Message ID")
    plt.ylabel("RTT (seconds)")
    plt.title("Round-Trip Time (RTT) Analysis with Packet Loss and Threshold")
    plt.grid(True)
    plt.legend()

    # Save the scatter plot as a PNG file
    plt.savefig(output_path)
    logger.info(f"Scatter plot saved as {output_path}")
    plt.close()

    # Histogram plot for RTT distribution
    plt.figure(figsize=(10, 6))
    plt.hist(rtts, bins=30, color='skyblue', edgecolor='black')
    plt.xlabel('RTT (seconds)')
    plt.ylabel('Frequency')
    plt.title('Distribution of RTTs')
    plt.grid(True)

    # Save the histogram as a PNG file
    histogram_output_path = output_path.replace(".png", "_histogram.png")
    plt.savefig(histogram_output_path)
    logger.info(f"Histogram plot saved as {histogram_output_path}")
    plt.close()



def write_statistics_to_csv(csv_file_path, statistics, test_duration, threshold_time):
    print("Appending data to csv")
    """Appends computed statistics in tabbed format to the CSV file."""
    with open(csv_file_path, "r", encoding="utf-8") as csvfile:
        lines = csvfile.readlines()

    # Find the location to insert computed statistics
    statistics_index = lines.index("# Computed Statistics\n") + 1

    # Preserve all lines including data rows before the statistics insertion point
    data_lines = lines[statistics_index:]

    # Construct the new statistics lines
    statistics_lines = [
        f"# Total Packets Sent       : {statistics['Total Packets Sent']}\n",
        f"# Packets Received         : {statistics['Packets Received']}\n",
        f"# Packets Lost             : {statistics['Packets Lost']}\n",
        f"# Packet Loss Percentage   : {statistics['Packet Loss Percentage']}\n",
        f"# Average RTT              : {statistics['Average RTT']}\n",
        f"# Median RTT               : {statistics['Median RTT']}\n",
        f"# RTT Range                : {statistics['RTT Range']}\n",
        f"# Standard Deviation of RTT: {statistics['Standard Deviation of RTT']}\n",
        f"# Variance of RTT          : {statistics['Variance of RTT']}\n",
        f"# Packets Under Threshold  : {statistics['Packets Under Threshold']}\n",
        f"# Packets Over Threshold   : {statistics['Packets Over Threshold']}\n",
        f"# Threshold RTT            : {threshold_time} s\n",
        f"# Test Duration            : {test_duration}\n",
        "#\n",
    ]

    # Combine all lines: original lines up to the statistics marker + new statistics + original data rows
    updated_lines = lines[:statistics_index] + statistics_lines + data_lines

    # Write back all the lines, including the original data and new statistics
    with open(csv_file_path, "w", encoding="utf-8") as csvfile:
        csvfile.writelines(updated_lines)


def main():
    print("Starting latency plotter")

    # Parse command-line arguments for threshold time
    parser = argparse.ArgumentParser(
        description="Process RTT logs and generate statistics and plots."
    )
    parser.add_argument(
        "--threshold", type=float, default=0.02, help="Threshold RTT time in seconds"
    )
    args = parser.parse_args()
    threshold_time = args.threshold

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

        # Check if the file has already been processed
        if is_processed(system_info):
            logger.info(f"Skipping already processed file: {csv_file}")
            continue

        if data:
            # Compute test duration
            test_duration = compute_test_duration(system_info, data)

            # Display data points and statistics in the terminal
            statistics = display_data(data, threshold_time)

            # Append computed statistics to CSV
            write_statistics_to_csv(
                csv_file_path, statistics, test_duration, threshold_time
            )

            # Plot the RTT graph and save as PNG
            output_path = os.path.join(
                data_folder, f"{os.path.splitext(csv_file)[0]}.png"
            )
            plot_rtt(data, output_path, threshold_time)
        else:
            logger.info(f"No data available for plotting from file '{csv_file}'.")
    print("Latency Plotter complete")


if __name__ == "__main__":
    main()
