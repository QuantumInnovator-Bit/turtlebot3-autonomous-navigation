# TurtleBot3 Autonomous Navigation

A comprehensive ROS2 Humble autonomous navigation system for TurtleBot3 robots, featuring obstacle avoidance, SLAM-based mapping, and path planning capabilities.

## Features

- **Reactive Obstacle Avoidance**: Simple LiDAR-based collision avoidance system
- **SLAM Integration**: Automatic map building using simultaneous localization and mapping
- **Nav2 Stack**: Full Navigation2 integration for advanced path planning
- **Gazebo Simulation**: Complete 3D physics-based simulation environment
- **RViz2 Visualization**: Real-time sensor data and navigation visualization
- **Modular Architecture**: Flexible launch configurations for different use cases

## System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS Distribution**: ROS2 Humble Hawksbill
- **Robot Platform**: TurtleBot3 (Burger/Waffle/Waffle Pi)
- **Simulator**: Gazebo 11
- **Visualization**: RViz2

## Installation

### Prerequisites

1. Install ROS2 Humble:
```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

2. Install TurtleBot3 packages:
```bash
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs
```

3. Set TurtleBot3 model environment variable:
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### Build Instructions

1. Clone this repository:
```bash
cd ~/ros2_ws/src
git clone https://github.com/QuantumInnovator-Bit/turtlebot3-autonomous-navigation.git
```

2. Build the workspace:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Option 1: Simple Obstacle Avoidance

Basic reactive navigation without advanced path planning:

**Terminal 1** - Launch Gazebo simulation:
```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_autonomous_nav gazebo_sim.launch.py
```

**Terminal 2** - Launch obstacle avoidance:
```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_autonomous_nav obstacle_avoidance_simple.launch.py
```

### Option 2: Advanced Navigation with SLAM

Full Nav2 stack with automatic map building:

**Terminal 1** - Launch Gazebo simulation:
```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_autonomous_nav gazebo_sim.launch.py
```

**Terminal 2** - Launch Nav2 with SLAM:
```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_autonomous_nav navigation.launch.py
```

### Option 3: Complete System

Launch everything in one command:
```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_autonomous_nav complete_system.launch.py
```

## Package Architecture

### Nodes

- **velocity_publisher**: Publishes velocity commands to `/cmd_vel` topic
  - Parameters: `linear_velocity`, `angular_velocity`
  
- **sensor_subscriber**: Processes odometry and LiDAR data
  - Subscribes to: `/odom`, `/scan`
  - Provides position tracking and obstacle detection
  
- **obstacle_avoidance**: Reactive collision avoidance
  - Parameters: `safe_distance`, `linear_speed`, `angular_speed`
  - Uses segmented LiDAR analysis for intelligent navigation

### Launch Files

- `gazebo_sim.launch.py`: Launches Gazebo with TurtleBot3 world
- `obstacle_avoidance_simple.launch.py`: Simple reactive navigation mode
- `navigation.launch.py`: Nav2 stack with SLAM and RViz2
- `complete_system.launch.py`: Full system (Gazebo + Nav2)

### Configuration

- `nav2_params.yaml`: Navigation2 stack parameters (costmaps, planners, controllers)
- `navigation.rviz`: RViz2 visualization configuration

## Navigation Features

### Obstacle Avoidance Algorithm
- Real-time LiDAR-based detection
- 360-degree coverage with segmented analysis (front, left, right)
- Configurable safe distance threshold
- Dynamic turning decisions based on available space

### SLAM Capabilities
- Automatic map generation during navigation
- AMCL-based localization
- Support for unknown environment exploration

### Path Planning
- Global path planning with NavFn planner
- Local trajectory optimization with DWB controller
- Dynamic obstacle avoidance with costmap layers

## RViz2 Usage

1. **Set Initial Pose**: Use "2D Pose Estimate" tool to set robot's starting position
2. **Set Navigation Goal**: Use "2D Goal Pose" tool to command the robot to a target location
3. **Monitor Progress**: View real-time sensor data, costmaps, and planned paths

## ROS2 Topic Interface

Key topics for monitoring and control:

```bash
# View all topics
ros2 topic list

# Monitor velocity commands
ros2 topic echo /cmd_vel

# View LiDAR data
ros2 topic echo /scan

# Check odometry
ros2 topic echo /odom

# Visualize node graph
rqt_graph
```

## Parameters

Customize behavior by modifying parameters:

```bash
# Set custom speeds for obstacle avoidance
ros2 run turtlebot3_autonomous_nav obstacle_avoidance --ros-args -p linear_speed:=0.3 -p angular_speed:=0.7

# Adjust safe distance threshold
ros2 run turtlebot3_autonomous_nav obstacle_avoidance --ros-args -p safe_distance:=0.6
```

## Troubleshooting

### Gazebo doesn't open
- Check DISPLAY variable: `echo $DISPLAY`
- For WSL: Install VcXsrv or X410 and set `export DISPLAY=:0`

### Robot doesn't move
- Verify obstacle_avoidance node is running: `ros2 node list`
- Check velocity commands: `ros2 topic echo /cmd_vel`
- Ensure Gazebo simulation is active

### Navigation issues
- Set initial pose in RViz2 using "2D Pose Estimate"
- Verify map is loaded and visible in RViz2
- Check Nav2 logs for errors: `ros2 topic echo /diagnostics`

## Project Structure

```
turtlebot3_autonomous_nav/
├── config/
│   ├── nav2_params.yaml          # Navigation2 parameters
│   └── navigation.rviz           # RViz2 configuration
├── launch/
│   ├── gazebo_sim.launch.py      # Gazebo simulation
│   ├── obstacle_avoidance_simple.launch.py
│   ├── navigation.launch.py      # Nav2 + SLAM
│   └── complete_system.launch.py
├── turtlebot3_autonomous_nav/
│   ├── velocity_publisher.py
│   ├── sensor_subscriber.py
│   └── obstacle_avoidance.py
├── package.xml
└── setup.py
```

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Acknowledgments

- Built on the ROS2 ecosystem and Navigation2 stack
- Uses TurtleBot3 robot platform
- Gazebo simulation environment
- RViz2 visualization tools

## Contact

For questions or support, please open an issue on the GitHub repository.

---

**Maintainer**: Robotics Developer  
**Email**: robotics@example.com
