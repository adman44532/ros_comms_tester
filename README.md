# ros_comms_tester

A testing package for communication within ROS2 and other systems like Unity or Godot. This repository aims to facilitate effective testing of communication endpoints and round-trip times (RTT) for messages between different systems and environments. 

## Overview

This repository provides tools for building, running, and analysing different communication metrics, especially for those involving complex systems like game engines. Here is a quick overview of the key capabilities:

- Test and evaluate RTT for message exchange.
- Integrate and test communication between ROS2 and game engines (Unity, Godot, UE5).
- Process and analyse communication test data for insights.

## Directory Structure

- **src/**
  - **comms_interfaces/**: Interfaces for communication tests.
  - **comms_tester/**: Main package containing test scripts and core logic.
    - **testers/**: Contains various test scripts, including round-trip time tests (`custom_message_rtt.py`, `simple_string_rtt.py`, etc.), emulators (`endpoint_emulator.py`), and the base node definition (`RTTBaseNode.py`).
  - **launch/**: Launch files for starting tests.
  - **resource/**: Resource files used during tests.
- **ROS-TCP-Endpoint/**: Used for handling ROS-TCP connections.
- **test/**: Contains configuration (`setup.cfg`, `package.xml`), and other support scripts (`latency_plotter.py`, `test_averager.py`).
- **data_storage/**: Directory where test data files are stored for analysis.

## Getting Started

### Building the Workspace

To build the workspace, navigate to the top-level directory and run:

```sh
colcon build
```

The `src` folder will be built to ensure the packages work with your system. Note that `COLCON_IGNORE` files are placed in other directories to avoid build conflicts with game engines.

### Setting Up an Endpoint

If you are testing a new plugin, ensure that the endpoint is successfully set up first within a new game engine project.

### Sourcing the Workspace

Source the setup file located in the `/install` folder to use the built packages:

```sh
source install/setup.bash
```

### Running Tests

To run a specific test, use the following command:

```sh
ros2 run comms_tester <TESTNAME>
```

A `/data` folder is created where the command is run, storing a `.csv` file that contains the RTT value for each message during communication.

### Data Storage

All collected data is stored in the `/data_storage` directory using the naming scheme `<device>-<device>_<PLUGIN>`. Copy all original unprocessed data files into a `RAW` folder before analysing the data.

## Data Processing Workflow

To process data in the `/data` folder, ensure that it only contains test data for a single scenario. The following scripts should then be run sequentially:

1. **Latency Plotter**:
   
   ```sh
   python3 latency_plotter.py
   ```
   This script plots latency information for easier visualisation of communication performance.

2. **Test Averager**:
   
   ```sh
   python3 test_averager.py
   ```
   This script calculates average RTT for the collected test data.

3. **High/Low Finder**:
   
   ```sh
   python3 high_low_finder.py
   ```
   This script helps to identify the highest and lowest RTTs in the collected data.

4. **Name Formatter (Optional)**:

   If any issues arise due to `:` in the naming convention, run:
   
   ```sh
   python3 name_formatter.py
   ```
   This script resolves naming issues related to special characters.

## Creating New Tests

### Using RTTBaseNode for ROS2 Tests

For creating new tests in ROS2, follow the implementation instructions provided in `RTTBaseNode.py` within the `comms_tester` package. The `RTTBaseNode` class provides predefined functions to help produce consistent outputs for tests compatible with `latency_plotter.py`.

### Creating New Metrics

If creating entirely new metrics to test, this will require a separate implementation outside of the `RTTBaseNode` class.

### Tests Outside the ROS2 Environment

For tests that involve metrics outside of the ROS2 environment, the new scripts should be placed in a separate directory at the top level. Ensure that instructions are included in the code on how to use the scripts effectively.

## Notes
- Ensure that the `/data` directory is cleared or organised after every test to avoid confusion when processing new data.
- If you're integrating with game engines like Unity or Godot, it's crucial to validate the ROS2-TCP connection beforehand.

## Contribution Guidelines

Feel free to contribute to this project by adding new tests, fixing bugs, or improving documentation. Please raise an issue or make a pull request.

---
This repository aims to simplify communication testing between ROS2 and game engines, providing all necessary tools for easy data collection and processing. If you have any questions or suggestions, please open an issue.
