# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
ASV-2025 is a ROS 2 based autonomous surface vehicle (ASV) system designed for competitive robotics. The system integrates multiple sensors, mission planning, navigation, and perception capabilities for autonomous operation.

## Build System & Commands

### Building the Project
```bash
# Build all packages
cd libs && colcon build --packages-up-to rplidar_ros serial vision_opencv

# Build main packages
cd main && colcon build

# Source the setup
source libs/install/setup.bash
source main/install/setup.bash
```

### Running the System
```bash
# Launch full mission system
ros2 launch launcher mission.launch.py

# Launch individual components
ros2 launch launcher mavros.launch.py
ros2 launch launcher vision.launch.py
ros2 launch launcher rplidar.launch.py
ros2 launch launcher remote.launch.py
```

### Testing
```bash
# Run Python tests with pytest
cd main/src/perception && python -m pytest test/

# Run flake8 linting
cd main/src/perception && python -m flake8 perception/
```

## Architecture Overview

### Core Packages Structure
- **libs/**: Third-party ROS packages (rplidar_ros, serial, vision_opencv)
- **main/src/**: Main application packages organized by functionality

### Main Application Packages
1. **launcher**: Central launch configuration and parameter management
2. **mission**: High-level mission planning and execution
3. **navigation**: GPS and vision-based navigation controllers
4. **perception**: Computer vision and sensor data processing
5. **sensor**: Hardware interface controllers (remote, lidar, state)
6. **monitoring**: System health monitoring (battery, CPU, GPS, GPU)

### Key System Components

#### Mission Planning (`mission/pose_planner.cpp`)
- Waypoint-based mission execution with configurable phase durations
- Supports two trajectory modes (A/B) for different mission profiles
- Integrates with MAVROS for flight controller communication
- Handles buoy detection coordination and navigation mode switching

#### Navigation System
- **GPS Navigation**: Waypoint following with configurable radius tolerance
- **Vision Navigation**: Computer vision-based docking and object avoidance
- **Lidar Integration**: Obstacle detection and avoidance using RPLidar

#### Perception Pipeline
- **Video Recording**: Continuous video capture for mission documentation
- **Object Detection**: Buoy and docking station detection
- **Visualization**: Separate surface and underwater visualization nodes

#### Hardware Integration
- **Remote Control**: Arduino-based RC input processing
- **Sensor Fusion**: GPS, IMU, and camera data integration
- **MAVROS Interface**: ArduPilot flight controller communication

### Configuration Management
Configuration files are centralized in `main/src/launcher/config/`:
- `mission.yaml`: Mission waypoints and phase durations
- `sensor.yaml`: Hardware interface parameters
- `vision.yaml`: Computer vision and PID control parameters

### Key Communication Topics
- `/mavros/*`: Flight controller communication
- `/robot/mission`: Current mission state
- `/robot/control/*`: Navigation mode control
- `/detect/*`: Object detection results
- `/robot/position/waypoint`: Current waypoint status

## Development Guidelines

### Code Organization
- C++ nodes for real-time control and mission critical components
- Python nodes for perception and non-critical processing
- Configuration-driven parameter management
- Modular launch file structure

### Testing Strategy
- Python packages use pytest with flake8 linting
- C++ packages use CMake with colcon build system
- Integration testing through launch files

### Common Development Tasks
When working on this codebase:
1. Always source the ROS 2 setup files before building or running
2. Use the configuration files for parameter changes rather than hardcoding
3. Follow the existing node structure and communication patterns
4. Test changes with individual launch files before using the full mission launch