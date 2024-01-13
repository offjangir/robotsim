# RobotSim

# CPU Monitor ROS 2 Node

This ROS 2 node monitors CPU usage for specified processes and publishes the data. It also provides a final plot of CPU usage over time.

## Requirements

- ROS 2 (Robot Operating System 2)
- Python 3
- `rclpy` library (ROS 2 Python client library)
- `psutil` library (cross-platform library for retrieving information on running processes and system utilization)
- `matplotlib` library (for plotting)

## Usage

1. Install the required dependencies:

   ```bash
   pip install rclpy psutil matplotlib
   ```
2. Running
    ```
    python3 cpu_usage.py
    ```

# Calculate Path Length Node

This ROS 2 node calculates the path length and traversal time using odometry data.

## Requirements

- ROS 2 (Robot Operating System 2)
- Python 3
- NumPy library (`numpy`)
- Standard ROS 2 message packages (`nav_msgs`, `builtin_interfaces`, `std_msgs`)

## Usage

1. Install the required dependencies:

   ```bash
   pip install numpy
   ```
2. Running
    ```
    python3 path_length.py
    ```


# Profile Publisher

This ROS 2 node monitors Turtlebot odometry and publishes various derived information such as velocity, acceleration, jerk, and total energy consumption.

## Requirements

- ROS 2 (Robot Operating System 2)
- Python 3
- Standard ROS 2 message packages (`nav_msgs`, `std_msgs`)


## Usage

1. Running
    ```
    python3 profile_pub.py
    ```



