# ASV-2025 Project Documentation

## Project Overview

ASV-2025 is a comprehensive ROS 2 autonomous surface vehicle (ASV) system designed for competitive robotics applications. The system provides integrated navigation, perception, and mission planning capabilities for autonomous operation in marine environments.

## Technology Stack

### Core Technologies

#### Operating System & Framework
- **ROS 2 Humble**: Primary robotics middleware framework
- **Ubuntu 22.04**: Recommended operating system
- **Python 3.10**: High-level scripting and AI/ML components
- **C++17**: Real-time control and performance-critical components

#### Computer Vision & AI
- **OpenCV 4.x**: Computer vision processing and image manipulation
- **YOLO (Ultralytics)**: Real-time object detection and classification
- **TensorRT**: GPU-accelerated inference optimization
- **cv_bridge**: OpenCV-ROS 2 integration library

#### Navigation & Control
- **MAVLink**: Communication protocol with flight controllers
- **GeographicLib**: Precise geodetic calculations and coordinate transforms
- **PID Control**: Proportional-Integral-Derivative control algorithms
- **Haversine Algorithm**: Great-circle distance calculations

#### Sensor Integration
- **RPLidar SDK**: LIDAR sensor interface and data processing
- **Serial Communication**: Hardware interface protocols
- **GPS/GNSS**: Global positioning and navigation
- **IMU Integration**: Inertial measurement unit data fusion

#### Build System & Dependencies
- **Colcon**: ROS 2 build system and package management
- **CMake**: C++ build configuration and compilation
- **setuptools**: Python package management
- **rosdep**: ROS dependency management

### Third-Party Libraries & Sources

#### Core Dependencies
- **MAVROS**: [mavlink/mavros](https://github.com/mavlink/mavros) (ros2 branch)
  - MAVLink communication library for ROS 2
  - Flight controller integration and protocol handling

- **Serial-ROS2**: [RoverRobotics-forks/serial-ros2](https://github.com/RoverRobotics-forks/serial-ros2)
  - Cross-platform serial communication library
  - Hardware interface abstraction

- **RPLidar-ROS**: [Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros) (ros2 branch)
  - LIDAR sensor driver and data processing
  - Obstacle detection and mapping

- **Vision-OpenCV**: [ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv) (humble branch)
  - OpenCV integration for ROS 2
  - Image processing and computer vision pipeline

#### Development Tools
- **GeographicLib Datasets**: Geoid models and coordinate reference systems
- **Python Colcon Extensions**: Enhanced build system capabilities
- **vcstool**: Version control system tools for multi-repository projects
- **rosinstall-generator**: Dependency resolution and installation

## System Architecture

### Core Structure

The project is organized into two main directories:

#### `libs/` - Third-Party Dependencies
- **rplidar_ros**: LIDAR sensor integration
- **serial**: Serial communication library
- **vision_opencv**: Computer vision and image processing
- **mavros**: MAVLink communication with flight controllers
- **cv_bridge**: OpenCV-ROS 2 bridge for image processing

#### `main/src/` - Application Packages
- **launcher**: Central launch configuration and parameter management
- **mission**: High-level mission planning and execution
- **navigation**: GPS and vision-based navigation controllers
- **perception**: Computer vision and sensor data processing
- **sensor**: Hardware interface controllers
- **monitoring**: System health monitoring

## Package Details

### 1. Mission Package

**Core Components**: 
- `gps_rtk_planner.cpp`: GPS RTK-based mission planning and execution
- `gps_vision_planner.cpp`: Vision-integrated mission planning (skeleton)

**Functionality**:
- RTK GPS-based mission execution with high precision positioning
- Supports dual trajectory modes (A/B) for different mission profiles
- Modern ROS2 parameter system with dotted notation
- Handles mission state transitions through waypoint progression

**Key Features**:
- Modern C++ patterns with `this->` convention
- ROS2 parameter system replacing YAML file reading
- Real-time mission state publishing
- Multi-phase mission execution with configurable waypoints

**Mission Phases**:
- **RTK GPS Navigation**: High-precision waypoint following
- **Vision Integration**: Combined GPS and vision-based navigation
- **Multi-trajectory Support**: Configurable A/B mission profiles
- **Parameter-driven**: ROS2 parameter system with dotted notation

### 2. Navigation Package

**Core Components**:
- `gps_nav.cpp`: GPS waypoint navigation with configurable radius tolerance
- `vision_nav.cpp`: Vision-based navigation and control

**Functionality**:
- GPS waypoint navigation with configurable radius tolerance
- ROS2 parameter system with dotted notation (`navigation.waypoints_A.count`)
- Haversine distance calculation for precise positioning
- Vision-based navigation integration

**Key Features**:
- Support for dual trajectory configurations (A/B)
- Modern C++ patterns with `this->` convention
- Real-time distance and waypoint status reporting
- Parameter-driven configuration system

### 3. Perception Package

**Core Components**:
- `vision.py`: YOLO-based object detection and vision processing
- `surface.cpp`: Surface imaging and processing
- `underwater.cpp`: Underwater imaging and processing
- `record.py`: Video recording and data logging

**Functionality**:
- YOLO-based object detection for buoys and docking stations
- PID-controlled vision navigation with RC override
- Multi-mission vision processing (Floating Ball, Surface/Underwater Imaging, Docking)
- Real-time video processing with FPS monitoring

**Key Features**:
- TensorRT model optimization for real-time inference
- Mission-specific object detection and tracking
- Configurable PID parameters for smooth navigation
- Surface and underwater imaging capture
- Integrated LIDAR coordination for precise positioning

### 4. Sensor Package

**Components**:
- **remote.cpp**: RC input processing and failsafe management
- **rplidar.cpp**: LIDAR data processing and obstacle detection
- **state.cpp**: System state monitoring and reporting
- **compass.cpp**: Compass-based heading control
- **gps.cpp**: GPS coordinate processing
- **emergency.cpp**: Emergency stop and safety protocols

### 5. Monitoring Package

**Components**:
- **battery.cpp**: Power management and voltage monitoring
- **cpu.cpp**: System resource monitoring
- **gps.cpp**: GPS signal quality and status
- **date.py**: System date/time publishing (converted from C++)

### 6. Launcher Package

**Configuration Management**:
- **mission.yaml**: Mission waypoints and phase timing configuration
- **sensor.yaml**: Hardware interface parameters
- **vision.yaml**: Computer vision and PID control parameters

**Launch Files**:
- **mission.launch.py**: Complete system launch with all components
- **mavros.launch.py**: MAVLink communication setup
- **vision.launch.py**: Vision processing components
- **rplidar.launch.py**: LIDAR sensor initialization
- **remote.launch.py**: RC input handling

## Communication Architecture

### Key Topics

#### Mission Control
- `/robot/mission`: Current mission state
- `/robot/control/gps`: GPS navigation enable/disable
- `/robot/control/compass`: Compass navigation enable/disable
- `/robot/control/vision`: Vision navigation enable/disable
- `/robot/position/waypoint`: Current waypoint status

#### Navigation
- `/mavros/global_position/global`: GPS position data
- `/mavros/setpoint_position/global`: GPS waypoint commands
- `/mavros/rc/override`: RC channel override for vision control

#### Perception
- `/robot/vision/navigation`: Processed navigation video
- `/robot/image/surface`: Surface imaging capture
- `/detect/left_buoy`: Left buoy detection status
- `/detect/right_buoy`: Right buoy detection status
- `/detect/docking`: Docking station detection

#### System Status
- `/robot/mode`: Auto/manual mode status
- `/robot/trajectory`: Current trajectory selection (A/B)
- `/mission/reset`: Mission reset command

## Build and Development

### Initial Setup

#### System Dependencies
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool python3-rosinstall-generator python3-osrf-pycommon
```

#### Third-Party Library Installation
```bash
cd libs/src

# Clone required repositories
git clone -b ros2 https://github.com/mavlink/mavros.git
git clone https://github.com/RoverRobotics-forks/serial-ros2.git
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
git clone -b humble https://github.com/ros-perception/vision_opencv.git

# Clean git metadata (optional)
find ./ -type d -name ".git" -exec rm -rf {} +
find ./ -type f -name ".gitmodules" -exec rm -f {} +
find ./ -type f -name ".gitignore" -exec rm -f {} +
```

#### MAVROS Geographic Datasets
```bash
cd libs
rosdep install --from-paths src --ignore-src -y
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm ./install_geographiclib_datasets.sh
```

### Build Commands
```bash
# Build dependencies
cd libs && colcon build --packages-up-to rplidar_ros serial vision_opencv mavros

# Build main application
cd main && colcon build

# Source environment
source libs/install/setup.bash
source main/install/setup.bash
```

### Launch Commands
```bash
# Full mission system
ros2 launch launcher mission.launch.py

# Individual components
ros2 launch launcher mavros.launch.py
ros2 launch launcher vision.launch.py
ros2 launch launcher rplidar.launch.py
ros2 launch launcher remote.launch.py
```

### Testing
```bash
# Python package testing (if test directory exists)
cd main/src/perception && python -m pytest test/ 2>/dev/null || echo "No pytest tests found"

# Code quality checks
cd main/src/perception && python -m flake8 perception/ --exclude=__pycache__
```

## Configuration

### Mission Configuration (`mission.yaml`)
- **Phase Duration**: Configurable timing for each mission phase
- **Waypoints**: GPS coordinates for trajectory A and B
- **Navigation Radius**: Waypoint proximity tolerance

### Sensor Configuration (`sensor.yaml`)
- **RC Parameters**: Remote control channel mapping
- **LIDAR Settings**: Sensor parameters and thresholds
- **Hardware Interfaces**: Serial port and communication settings

### Vision Configuration (`vision.yaml`)
- **PID Parameters**: Vision navigation control gains
- **Model Configuration**: YOLO detection model settings
- **Camera Settings**: Resolution and capture parameters

## Hardware Integration

### Supported Hardware
- **Flight Controller**: ArduPilot-compatible autopilot via MAVLink
- **LIDAR**: RPLidar A1/A2/A3 series
- **Camera**: USB camera for vision processing
- **RC Receiver**: Standard RC input for manual override
- **GPS**: MAVLink-compatible GPS module

### Communication Protocols
- **MAVLink**: Flight controller communication
- **Serial**: Sensor data acquisition
- **USB**: Camera video capture
- **Network**: ROS 2 inter-process communication

## Development Guidelines

### Code Organization
- **C++ Components**: Real-time control and mission-critical functions using modern `this->` convention
- **Python Components**: Perception and non-critical processing with modern ROS2 patterns
- **Parameter-Driven**: ROS2 parameter system with dotted notation replacing YAML files
- **Mixed Package Support**: ament_cmake_python for packages with both C++ and Python

### Best Practices
- Modern C++ patterns with `this->` member access convention
- ROS2 parameter system with `declare_parameter()` and `get_parameter()`
- In-class initialization to reduce constructor complexity
- Member variables organized at bottom of private sections
- Comprehensive error handling and logging
- Systematic testing with individual component launch files

### Integration Testing
- Component-level testing through individual launch files
- System-level integration via complete mission launch
- Hardware-in-the-loop testing with actual sensors
- Simulation testing for algorithm validation

## Performance Characteristics

### Real-Time Requirements
- **Mission Planning**: 20ms update cycle (50 Hz)
- **Navigation Control**: 20ms update cycle (50 Hz)
- **Vision Processing**: 20 Hz with TensorRT optimization
- **Sensor Monitoring**: 1 Hz for non-critical systems

### Resource Utilization
- **CPU**: Optimized for multi-core processing
- **Memory**: Efficient image processing with cv_bridge
- **GPU**: TensorRT acceleration for vision inference
- **Network**: Minimal latency ROS 2 communication

## Safety and Reliability

### Failsafe Mechanisms
- **RC Override**: Manual control takeover capability
- **Emergency Stop**: Immediate system shutdown
- **State Monitoring**: Continuous system health checks
- **Mission Reset**: Complete mission state reset capability

### Error Handling
- **Parameter Validation**: YAML configuration validation
- **Connection Monitoring**: Hardware connection status
- **Graceful Degradation**: Reduced functionality on component failure
- **Logging**: Comprehensive system event logging

## Future Enhancements

### Planned Features
- **Multi-Vehicle Coordination**: Fleet management capabilities
- **Advanced Perception**: Enhanced object detection and tracking
- **Machine Learning**: Adaptive navigation algorithms
- **Simulation Integration**: Gazebo-based testing environment

### Scalability Considerations
- **Modular Architecture**: Easy component addition/removal
- **Configuration Management**: Centralized parameter system
- **Performance Optimization**: GPU acceleration and parallel processing
- **Documentation**: Comprehensive developer documentation