# TurtleBot3 Autonomous Navigation - ROS2 Humble Project

## Overview

This project implements an autonomous navigation system for TurtleBot3 robots using ROS2 Humble. It provides obstacle avoidance capabilities, sensor data processing, and integration with Gazebo simulation and RViz2 visualization. The system is designed for Ubuntu 22.04 and includes multiple launch configurations for different operational modes, from simple obstacle avoidance to complete autonomous navigation with SLAM (Simultaneous Localization and Mapping).

## User Preferences

Preferred communication style: Simple, everyday language.

## System Architecture

### ROS2 Package Architecture

**Problem:** Need a modular, maintainable structure for autonomous robot navigation capabilities.

**Solution:** Python-based ROS2 package using standard ROS2 patterns with separate nodes for distinct responsibilities:
- `velocity_publisher`: Controls robot movement by publishing velocity commands
- `sensor_subscriber`: Processes sensor data from odometry and LIDAR
- `obstacle_avoidance`: Implements reactive obstacle avoidance logic

**Rationale:** This separation of concerns allows independent development and testing of each capability. Each node can be launched individually or combined, providing flexibility in system configuration.

### Node Communication Pattern

**Problem:** Nodes need to share sensor data and coordinate robot control.

**Solution:** Pub/Sub architecture using ROS2 topics:
- `/cmd_vel`: Twist messages for velocity commands (published by velocity_publisher and obstacle_avoidance)
- `/scan`: LaserScan messages for LIDAR data (subscribed by sensor_subscriber and obstacle_avoidance)
- `/odom`: Odometry messages for robot position (subscribed by sensor_subscriber)

**Rationale:** ROS2's topic-based messaging provides loose coupling between nodes, making the system more resilient and easier to extend. Multiple nodes can subscribe to the same sensor data without modifying existing code.

### Obstacle Avoidance Algorithm

**Problem:** Robot needs to navigate autonomously while avoiding collisions.

**Solution:** Reactive obstacle avoidance using segmented LIDAR analysis:
- Divides LIDAR readings into front, left, and right sectors
- Maintains configurable safe distance threshold (default 0.5m)
- Turns toward the side with more clearance when obstacles detected
- Uses parameterized speeds for easy tuning

**Rationale:** This reactive approach provides immediate response to obstacles without requiring complex path planning. The segmented analysis allows the robot to make intelligent turning decisions based on available space.

### Launch File Hierarchy

**Problem:** Different use cases require different combinations of nodes and external systems.

**Solution:** Layered launch file structure:
- `obstacle_avoidance_simple.launch.py`: Basic obstacle avoidance without navigation stack
- `gazebo_sim.launch.py`: Gazebo simulation environment setup
- `navigation.launch.py`: Full Nav2 stack with SLAM and RViz2
- `complete_system.launch.py`: Combines Gazebo simulation with navigation stack

**Rationale:** This hierarchy supports progressive complexity, allowing users to start simple and add capabilities as needed. It also facilitates debugging by isolating different system components.

### Parameter Management

**Problem:** System behavior needs to be tunable without code changes.

**Solution:** ROS2 parameter system for runtime configuration:
- Node-level parameters declared in Python (e.g., safe_distance, linear_speed)
- System-level parameters in YAML files (nav2_params.yaml for navigation stack)
- Parameters can be set via launch files or command line

**Rationale:** This approach separates configuration from implementation, enabling the same code to work in different scenarios with adjusted parameters.

### Simulation-First Development

**Problem:** Testing on physical robots is time-consuming and risky during development.

**Solution:** Gazebo-based simulation as primary development environment:
- All launch files support `use_sim_time` parameter
- Nodes work identically in simulation and on real hardware
- TurtleBot3 world provides standardized test environment

**Rationale:** Simulation accelerates development cycles and enables testing of dangerous scenarios safely. The simulation/real hardware compatibility ensures that tested code will work on physical robots.

## External Dependencies

### ROS2 Humble Framework
The foundational robotics middleware providing:
- Node lifecycle management
- Topic-based pub/sub messaging
- Parameter server functionality
- Launch system for orchestrating multiple processes
- Required for all core functionality

### TurtleBot3 Packages (`turtlebot3_gazebo`, `turtlebot3_*`)
Official TurtleBot3 support packages providing:
- Robot model definitions (URDF)
- Gazebo simulation models
- Pre-configured launch files for robot state publishing
- Hardware abstraction layer
- Essential for robot compatibility

### Navigation2 (Nav2) Stack
Complete autonomous navigation solution including:
- AMCL for localization
- Local and global path planning
- Costmap generation for obstacle representation
- Behavior trees for navigation logic
- Required for advanced autonomous navigation features

### Gazebo Simulator
3D physics-based robot simulator providing:
- Realistic sensor simulation (LIDAR, odometry)
- Physics engine for robot dynamics
- Virtual environment rendering
- Used for development and testing without physical hardware

### RViz2
3D visualization tool for ROS2 providing:
- Real-time sensor data visualization
- Robot model display
- Map and path visualization
- Interactive goal setting for navigation
- Essential for monitoring and debugging

### Python Dependencies
- `setuptools`: Package installation and distribution
- `ament_index_python`: Package resource location
- `rclpy`: Python client library for ROS2
- `math`: Standard library for geometric calculations

All packages are managed through the ROS2 ecosystem and Ubuntu's apt package manager, with specific versions tied to ROS2 Humble compatibility on Ubuntu 22.04.