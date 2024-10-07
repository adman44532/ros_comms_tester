import os
import datetime
import json

data_storage_path = "data_storage"

# Environment is a particular plugin on a particular device/pair
# Test type is one of the ones written in the comms_tester package

# Create a dictionary to store averages for each environment and test type
results = {}

# Only selected statistics that are easy to average, other ones either didn't have % or not relevant
computed_statistics = [
    "Packet Loss Percentage",
    "Average RTT",
    "Median RTT",
    "Standard Deviation of RTT",
    "Variance of RTT"
]

def get_test_type(file_name):
    if "simple_string" in file_name:
        return "simple_string"
    elif "large_payload" in file_name:
        return "large_payload"
    elif "increasing_payload" in file_name:
        return "increasing_payload"
    return "unknown"

# Traverse the data_storage folder, only considering CSV files in folders directly under the root
for root, dirs, files in os.walk(data_storage_path):
    if root == data_storage_path:
        for directory in dirs:
            dir_path = os.path.join(root, directory)
            for file in os.listdir(dir_path):
                if file.endswith(".csv"):
                    file_path = os.path.join(dir_path, file)
                    environment = directory
                    test_type = get_test_type(file)

                    if environment not in results:
                        results[environment] = {}
                    if test_type not in results[environment]:
                        results[environment][test_type] = {stat: [] for stat in computed_statistics}

                    # Read CSV file and compute statistics
                    try:
                        with open(file_path, 'r') as f:
                            for line in f:
                                if line.startswith('# ') and any(stat in line for stat in computed_statistics):
                                    for stat in computed_statistics:
                                        if stat in line:
                                            value_str = line.split(':')[-1].strip().split()[0]
                                            value = float(value_str.replace('%', '')) if '%' in value_str else float(value_str)
                                            results[environment][test_type][stat].append(value)
                    except Exception as e:
                        print(f"Error reading file {file_path}: {e}")

# Calculate the average for each computed statistic in each environment and test type
summary = {}
for environment, test_types in results.items():
    summary[environment] = {}
    for test_type, stats in test_types.items():
        summary[environment][test_type] = {}
        for stat, values in stats.items():
            if values:
                overall_avg = sum(values) / len(values)
                summary[environment][test_type][stat] = overall_avg

# Write the results to a new JSON file
json_output_file = os.path.join(
    data_storage_path,
    f"latency_summary_{datetime.datetime.now().strftime('%Y-%m-%d %H_%M_%S')}.json"
)
with open(json_output_file, 'w') as f:
    json.dump(summary, f, indent=4)

# Write the results to a new Markdown file
markdown_output_file = os.path.join(
    data_storage_path,
    f"latency_summary_{datetime.datetime.now().strftime('%Y-%m-%d %H_%M_%S')}.md"
)
with open(markdown_output_file, 'w') as f:
    for environment, test_types in summary.items():
        f.write(f"## Environment: {environment}\n")
        for test_type, stats in test_types.items():
            f.write(f"### Test Type: {test_type}\n")
            for stat, avg in stats.items():
                f.write(f"- **{stat}**: {avg:.6f}\n")
        f.write("\n")

# Write the results to a new CSV file
csv_output_file = os.path.join(
    data_storage_path,
    f"latency_summary_{datetime.datetime.now().strftime('%Y-%m-%d %H_%M_%S')}.csv"
)
with open(csv_output_file, 'w') as f:
    f.write("Environment,Test Type,Statistic,Average Value\n")
    for environment, test_types in summary.items():
        for test_type, stats in test_types.items():
            for stat, avg in stats.items():
                f.write(f"{environment},{test_type},{stat},{avg:.6f}\n")

# Print the results
for environment, test_types in summary.items():
    print(f"Environment: {environment}")
    for test_type, stats in test_types.items():
        print(f"  Test Type: {test_type}")
        for stat, avg in stats.items():
            print(f"    {stat}: {avg:.6f}")
    print()
