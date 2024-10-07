# This is an analysis tool to calculate a variety of statistics based on the RTT tests.
#
# File: comms_tester/latency_plotter.py

import os
import logging
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
from concurrent.futures import ProcessPoolExecutor
import shutil

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def read_csv_data(csv_file_path):
    print("Reading CSV Data")
    """Reads RTT data from the CSV file, skipping system information lines, and returns it as DataFrame."""
    if not os.path.exists(csv_file_path):
        logger.error(f"The file '{csv_file_path}' does not exist.")
        return None, None

    # Read CSV using Pandas, separate system info and data
    with open(csv_file_path, mode="r") as file:
        lines = file.readlines()

    system_info = [line for line in lines if line.startswith("#")]
    data_lines = ''.join([line for line in lines if not line.startswith("#")])

    if not data_lines:
        logger.error(f"No data found in the file '{csv_file_path}'.")
        return system_info, None

    # Create DataFrame from data lines
    data = pd.read_csv(pd.io.common.StringIO(data_lines))
    return system_info, data

def is_processed(system_info):
    """Checks if the CSV file has already been processed by looking for computed statistics."""
    return any("# Total Packets Sent" in line for line in system_info)

def compute_test_duration(system_info, data):
    """Computes the test duration based on the start time and last data point."""
    start_time_line = next((line for line in system_info if "Start Time" in line), None)
    if not start_time_line:
        return "0 seconds"

    start_time_str = start_time_line.split(":", 1)[1].strip()
    start_dt = datetime.strptime(start_time_str, "%Y-%m-%d %H:%M:%S")

    if data.empty:
        return "0 seconds"

    if 'receive_time' in data and not data['receive_time'].dropna().empty:
        end_time = data['receive_time'].dropna().iloc[-1]
    elif 'send_time' in data and not data['send_time'].empty:
        end_time = data['send_time'].iloc[-1]
    else:
        return "0 seconds"

    end_dt = start_dt + timedelta(seconds=end_time - data['send_time'].iloc[0])
    duration = end_dt - start_dt
    return str(duration).split(".")[0]

def display_data(data, threshold_time):
    print("Computing statistics")
    """Displays all relevant data points and statistics in the terminal, including packet loss information."""
    total_packets = len(data)
    received_data = data.dropna(subset=['rtt'])
    lost_packets = total_packets - len(received_data)
    packet_loss_percentage = (lost_packets / total_packets) * 100

    packets_under_threshold = (received_data['rtt'] <= threshold_time).sum()
    packets_over_threshold = (received_data['rtt'] > threshold_time).sum()
    percent_under_threshold = (packets_under_threshold / total_packets) * 100
    percent_over_threshold = (packets_over_threshold / total_packets) * 100

    statistics = {
        "Total Packets Sent": total_packets,
        "Packets Received": len(received_data),
        "Packets Lost": lost_packets,
        "Packet Loss Percentage": f"{packet_loss_percentage:.2f}%",
        "Average RTT": received_data['rtt'].mean() if not received_data.empty else "N/A",
        "Median RTT": received_data['rtt'].median() if not received_data.empty else "N/A",
        "RTT Range": (received_data['rtt'].max() - received_data['rtt'].min()) if not received_data.empty else "N/A",
        "Standard Deviation of RTT": received_data['rtt'].std() if not received_data.empty else "N/A",
        "Variance of RTT": received_data['rtt'].var() if not received_data.empty else "N/A",
        "Packets Under Threshold": packets_under_threshold,
        "Packets Over Threshold": packets_over_threshold,
        "% Under Threshold": f"{percent_under_threshold:.2f}%",
        "% Over Threshold": f"{percent_over_threshold:.2f}%",
    }

    for key, value in statistics.items():
        logger.info(f"{key: <30}: {value}")

    return statistics

def plot_rtt(data, output_path, threshold_time):
    print("Creating plots")
    """Plots the RTT graph using matplotlib, marks packet loss and over-threshold packets, and saves the plot as a PNG file."""
    received_data = data.dropna(subset=['rtt'])
    lost_data = data[data['rtt'].isna()]

    if received_data.empty:
        logger.info("No valid RTT data to plot.")
        return

    plt.figure(figsize=(12, 6))

    # Plot RTTs for packets under threshold
    under_threshold = received_data[received_data['rtt'] <= threshold_time]
    over_threshold = received_data[received_data['rtt'] > threshold_time]

    plt.scatter(under_threshold['message_id'], under_threshold['rtt'], marker="o", color="b", label="Meets Threshold")
    plt.scatter(over_threshold['message_id'], over_threshold['rtt'], marker="o", color="orange", label="Over Threshold")
    plt.scatter(lost_data['message_id'], [0] * len(lost_data), color="r", marker="x", s=100, label="Lost Packets")

    plt.axhline(y=threshold_time, color="g", linestyle="--", label=f"Threshold RTT = {threshold_time:.4f}s")

    if len(received_data) > 1:
        coeffs = np.polyfit(received_data['message_id'], received_data['rtt'], 1)
        best_fit_line = np.poly1d(coeffs)
        plt.plot(received_data['message_id'].to_numpy(), best_fit_line(received_data['message_id'].to_numpy()), color="red", linestyle="-", linewidth=2, label="Line of Best Fit")

    plt.xlabel("Message ID")
    plt.ylabel("RTT (seconds)")
    plt.title("Round-Trip Time (RTT) Analysis with Packet Loss and Threshold")
    plt.grid(True)
    plt.legend()
    plt.savefig(output_path)
    logger.info(f"Scatter plot saved as {output_path}")
    plt.close()

    # Create a histogram for RTT distribution
    plt.figure(figsize=(10, 6))
    plt.hist(received_data['rtt'], bins=30, color="skyblue", edgecolor="black")
    plt.xlabel("RTT (seconds)")
    plt.ylabel("Frequency")
    plt.title("Distribution of RTTs")
    plt.grid(True)

    histogram_output_path = output_path.replace(".png", "_histogram.png")
    plt.savefig(histogram_output_path)
    logger.info(f"Histogram plot saved as {histogram_output_path}")
    plt.close()

def write_statistics_to_csv(output_csv_path, system_info, statistics, test_duration, threshold_time, data):
    print("Writing data to new csv")
    """Writes the original data and computed statistics to a new CSV file."""
    with open(output_csv_path, "w", encoding="utf-8") as csvfile:
        csvfile.writelines(system_info)
        csvfile.write("# Computed Statistics\n")
        for key, value in statistics.items():
            csvfile.write(f"# {key: <30}: {value}\n")
        csvfile.write(f"# Threshold RTT                    : {threshold_time} s\n")
        csvfile.write(f"# Test Duration                   : {test_duration}\n")
        csvfile.write("#\n")
    data.to_csv(output_csv_path, mode='a', index=False)

def process_csv_file(csv_file_path, threshold_time, output_folder):
    system_info, data = read_csv_data(csv_file_path)

    if system_info is None or data is None:
        return

    if is_processed(system_info):
        logger.info(f"Skipping already processed file: {os.path.basename(csv_file_path)}")
        return

    test_duration = compute_test_duration(system_info, data)
    statistics = display_data(data, threshold_time)
    output_csv_path = os.path.join(output_folder, os.path.basename(csv_file_path))
    write_statistics_to_csv(output_csv_path, system_info, statistics, test_duration, threshold_time, data)
    output_png_path = os.path.join(output_folder, f"{os.path.splitext(os.path.basename(csv_file_path))[0]}.png")
    plot_rtt(data, output_png_path, threshold_time)

def main():
    print("Starting latency plotter")
    parser = argparse.ArgumentParser(description="Process RTT logs and generate statistics and plots.")
    parser.add_argument("--threshold", type=float, default=0.02, help="Threshold RTT time in seconds")
    args = parser.parse_args()
    threshold_time = args.threshold

    data_storage_folder = "data_storage"

    if not os.path.exists(data_storage_folder):
        logger.error(f"The data storage folder '{data_storage_folder}' does not exist.")
        return

    environments = [env for env in os.listdir(data_storage_folder) if os.path.isdir(os.path.join(data_storage_folder, env))]

    for environment in environments:
        env_folder = os.path.join(data_storage_folder, environment)
        raw_folder = os.path.join(env_folder, "RAW")
        output_folder = env_folder

        if not os.path.exists(raw_folder):
            logger.error(f"The RAW folder for environment '{environment}' does not exist.")
            continue

        # Delete existing computed results including .png files in the environment folder
        for file in os.listdir(output_folder):
            if file.endswith(".csv") or file.endswith(".png"):
                file_path = os.path.join(output_folder, file)
                if os.path.isfile(file_path):
                    os.remove(file_path)
                    logger.info(f"Deleted file: {file_path}")

        # Get all CSV files in the RAW folder
        csv_files = [os.path.join(raw_folder, f) for f in os.listdir(raw_folder) if f.endswith(".csv")]

        if not csv_files:
            logger.info(f"No CSV files found in the RAW folder for environment '{environment}'.")
            continue

        # Use ProcessPoolExecutor to process CSV files in parallel
        with ProcessPoolExecutor() as executor:
            futures = [executor.submit(process_csv_file, csv_file, threshold_time, output_folder) for csv_file in csv_files]
            for future in futures:
                future.result()

    print("Latency Plotter complete")

if __name__ == "__main__":
    main()