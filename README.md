# ROS 2 Working with WeBots - Programming Assignment 4 

A ROS2 package implementing a Roomba-like walker algorithm for obstacle avoidance using the State design pattern. The robot moves forward until it encounters an obstacle, then rotates in place (alternating between clockwise and counterclockwise) until the path is clear.

## Overview

This package contains:
- **Walker Algorithm**: State machine-based obstacle avoidance using the State design pattern
- **ROS2 Node**: `obstacle_avoider` node that implements the walker behavior
- **Webots Integration**: Webots plugin for robot control in simulation
- **Launch File**: Complete launch configuration with optional rosbag recording

### Architecture

The walker uses a finite state machine with two states:
- **MovingForwardState**: Robot moves forward when no obstacles are detected
- **RotatingState**: Robot rotates in place when obstacles are detected, alternating rotation direction

## Package Structure

```
walker/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/walker/
│   ├── walker_logic.hpp          # State pattern implementation
│   ├── obstacle_avoider.hpp      # ROS2 node header
│   ├── walker_node.hpp           # Webots plugin header
│   └── motor_controller.hpp      # Motor control utility
├── src/
│   ├── walker_logic.cpp          # State machine implementation
│   ├── obstacle_avoider.cpp      # ROS2 node implementation
│   └── walker_node.cpp           # Webots plugin implementation
├── launch/
│   └── walker_launch.py          # Main launch file
├── resource/
│   └── walker_robot.urdf         # Robot URDF description
├── worlds/
│   └── turtlebot3_burger_example.wbt  # Webots world file
└── results/                      # Bag files saved here (created automatically)
```

## Dependencies and Assumptions

### Required Dependencies
- **ROS2 Humble** (tested on Ubuntu 22.04)
- **Webots** (for simulation)
- **webots_ros2_driver** package
- **rosbag2** (for bag file recording/playback)

### ROS2 Packages
- `rclcpp`
- `geometry_msgs`
- `sensor_msgs`
- `webots_ros2_driver`
- `pluginlib`

## Setup (Build & Run)

1. Navigate to your ROS2 workspace root (`tutorials_ws`):
   ```bash
   cd ~/tutorials_ws/src
   ```
   or create one using:
   ```bash
   mkdir -p ~/tutorials_ws/src
   cd ~/tutorials_ws/src
   ```

2. Clone or copy the walker package into your workspace:
   ```bash
   git clone https://github.com/Madhav2133/my_webots_tutorials.git
   ```

3. Source your ROS2 environment (if not already sourced):
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. Build the package:
   ```bash
   # go to the workspace directory
   cd ~/tutorial_ws/
   colcon build 
   ```

5. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Simulation

### Basic Launch (with bag recording enabled by default)

```bash
ros2 launch walker walker_launch.py
```

This will:
- Launch Webots simulation with the walker robot
- Start the obstacle avoidance node
- Begin recording rosbag files to `src/walker/results/walker_bag` (default)

### Launch with Custom Bag Path

To save bag files to a custom location:

```bash
ros2 launch walker walker_launch.py bag_path:=/path/to/your/custom/location
```

### Disable Bag File Recording

To run the simulation without recording:

```bash
ros2 launch walker walker_launch.py record_bag:=false
```

### Launch Arguments

- `record_bag` (default: `true`): Enable/disable rosbag recording
  - `true`: Record all topics (except `/camera/*`)
  - `false`: Disable recording

- `bag_path` (default: `src/walker/results/walker_bag`): Path where rosbag files will be saved
  - Can be relative (from workspace root) or absolute path

## Rosbag File Operations

### Recording Bag Files

Bag files are automatically recorded when `record_bag:=true` (default). The recording includes:
- All ROS2 topics **except** `/camera/*` topics (to reduce file size)
- Topics such as `/cmd_vel`, `/left_sensor`, `/right_sensor`, `/tf`, `/tf_static`, etc.

**Note**: Camera topics are excluded because RGB-D sensor data significantly increases bag file size. (there are no camera topics in my robot)

### Inspecting Bag Files

To see what topics are recorded in a bag file:

```bash
ros2 bag info <path_to_bag>
```

You can replace <path_to_bag> with your bag's path like:

```bash
ros2 bag info src/my_webots_tutorials/results/walker_bag/
```


This will display:
- Bag file duration
- Number of messages per topic
- Topic names and message types
- Storage format

### Example Output:
```c#
Files:             walker_bag_0.db3
Bag size:          1.2 MiB
Storage id:        sqlite3
Duration:          83.950287925s
Start:             Nov 26 2025 18:42:14.844922740 (1764200534.844922740)
End:               Nov 26 2025 18:43:38.795210665 (1764200618.795210665)
Messages:          15106
Topic information: Topic: /right_sensor | Type: sensor_msgs/msg/Range | Count: 3766 | Serialization Format: cdr
                   Topic: /remove_urdf_robot | Type: std_msgs/msg/String | Count: 1 | Serialization Format: cdr
                   Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 8 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 7534 | Serialization Format: cdr
                   Topic: /left_sensor | Type: sensor_msgs/msg/Range | Count: 3766 | Serialization Format: cdr
                   Topic: /events/write_split | Type: rosbag2_interfaces/msg/WriteSplitEvent | Count: 0 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 31 | Serialization Format: cdr
```

### Playing Back Bag Files

**Important**: Webots should **NOT** be running when playing back bag files.

1. Make sure Webots is not running (close any active Webots instances)

2. Source your ROS2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

3. Play back the bag file:
   ```bash
   ros2 bag play src/my_webots_tutorials/results/walker_bag/
   ```

4. In another terminal, you can monitor topics:
   ```bash
   # Monitor velocity commands
   ros2 topic echo /cmd_vel
   
   # Monitor sensor readings
   ros2 topic echo /left_sensor
   ros2 topic echo /right_sensor
   ```

### Advanced Bag Playback Options

Play at a specific rate (e.g., 2x speed):
```bash
ros2 bag play src/my_webots_tutorials/results/walker_bag/ --rate 2.0
```

Play only specific topics:
```bash
ros2 bag play src/my_webots_tutorials/results/walker_bag/ --topics /cmd_vel /left_sensor /right_sensor
```

Start playback from a specific time offset:
```bash
ros2 bag play src/my_webots_tutorials/results/walker_bag/ --start-offset 5.0  # Start 5 seconds in
```



## Testing

The package is designed to support Level 1 and Level 2 unit testing:
- **Level 1**: Test individual classes (`WalkerLogic`, `MotorController`) in isolation
- **Level 2**: Test ROS2 node integration with mocked publishers/subscribers

## License

Apache-2.0

