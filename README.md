# Project Documentation

## Project Overview

LSS-02 is a comprehensive ROS 2 autonomous surface vehicle (ASV) system designed for competitive robotics applications. The system provides integrated navigation, perception, and mission planning capabilities for autonomous operation in marine environments.

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
- **RKNN Runtime**: NPU-accelerated inference optimization for Orange Pi 5 Pro
- **TensorRT**: GPU-accelerated inference optimization (legacy support)
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

### Struktur Folder Project (Project Folder Structure)

```
ASV-2025/
├── docsdev/                         # Dokumentasi & riset pengembangan (Vision, LIDAR, Algoritma, perbandingan)
│   ├── algoritm.md
│   ├── comparison.md
│   ├── lidar.md
│   ├── materi.md
│   ├── multiobjectrack.md
│   ├── update.md
│   └── vision_optimization.md
├── libs/                            # Third-Party ROS 2 Packages & Dependencies
│   └── src/
│       ├── mavros/                  # MAVLink communication interface
│       ├── rplidar_ros/             # Driver LIDAR Slamtec RPLidar
│       ├── serial-ros2/             # Serial communication library
│       └── vision_opencv/           # cv_bridge & image_geometry
├── main/                            # Core ROS 2 Workspace Application
│   └── src/
│       ├── launcher/                # Package peluncur (Launch files, YAML configs, Debug scripts)
│       │   ├── config/              # Parameter YAML (perception.yaml, sensor.yaml, navigation/, mission/)
│       │   ├── debug/               # Bash script penguji & penganalisis mode
│       │   ├── launch/              # Launch files ROS 2 (mission, standalone, rosbridge, mavros)
│       │   └── legacy/              # Skrip pendukung/arsip terdahulu
│       ├── mission/                 # Master Mission Control
│       │   └── mission/
│       │       └── gps_compass4_planner.cpp  # Planner state-machine navigasi utama
│       ├── navigation/              # Pengendali Navigasi (Global & Local)
│       │   └── navigation/
│       │       ├── compass_nav.cpp  # Navigasi kompas/heading lokal
│       │       ├── gps_nav.cpp      # Navigasi koordinat global (GPS Waypoints)
│       │       └── vision_nav.cpp   # Navigasi visual reaktif (buoy / pelampung PID)
│       ├── perception/              # Computer Vision & Pengolahan Kamera
│       │   └── perception/
│       │       ├── models/          # Bobot model AI NPU (RKNN model .rknn & .pt)
│       │       ├── utils/           # Utility script & novisual helper
│       │       ├── recorder.py      # Perekam video/frame kamera
│       │       ├── underwater.cpp   # Pengolahan citra bawah air
│       │       └── vision.py        # Deteksi objek YOLO terakselerasi RKNN Orange Pi 5 Pro
│       ├── sensor/                  # Interface Sensor & Komunikasi Hardware
│       │   └── sensor/
│       │       ├── livox.cpp        # Interface sensor LiDAR Livox
│       │       ├── remote.cpp       # Interface RC Remote Control
│       │       ├── rplidar.cpp      # Pengolahan rintangan RPLidar
│       │       └── state.cpp        # Pengawas status sistem hardware
│       └── monitoring/              # Monitoring Kesehatan Sistem & Resource
│           └── monitoring/
│               ├── battery.cpp      # Monitoring tegangan daya baterai
│               ├── cpu.cpp          # Resource CPU & suhu
│               ├── date.py          # Sinkronisasi & waktu sistem
│               └── gps.cpp          # Status & kualitas sinyal GPS
├── Testing/                         # Modul pengujian model AI & dataset lokal
│   ├── Detection.py
│   ├── best.pt
│   └── yolo11n.pt
└── web_ui/                          # Interface Web Dashboard Monitoring & Control (Next.js & ROSBridge)
    ├── src/
    ├── public/
    └── package.json
```

---

## Package Details

### 1. Mission Package (`main/src/mission`)

**Core Components**: 
- `gps_compass4_planner.cpp`: State-machine pengendali urutan misi (Global Planner).

**Functionality**:
- Mengoordinasikan fasa navigasi (*Floating Ball*, *Surface Imaging*, *Underwater Imaging*).
- Mengontrol perpindahan mode otomatis antar modul `gps_nav`, `compass_nav`, dan `vision_nav`.
- Berkomunikasi via ROS 2 topic `/robot/mission`, `/robot/control/*`, dan `/robot/setmode`.

---

### 2. Navigation Package (`main/src/navigation`)

**Core Components**:
- `gps_nav.cpp`: Navigasi Waypoint GPS Global (Haversine distance calculation & setpoint global).
- `compass_nav.cpp`: Navigasi penyesuaian sudut orientasi (heading kompas lokal).
- `vision_nav.cpp`: Navigasi visual reaktif berbasis PID override kontrol RC untuk mengunci buoy.

---

### 3. Perception Package (`main/src/perception`)

**Core Components**:
- `vision.py`: Pengolah deteksi YOLO terakselerasi NPU RKNN (Orange Pi 5 Pro) dengan arsitektur multi-thread.
- `underwater.cpp`: Pengolahan citra bawah air (*underwater capture/imaging*).
- `recorder.py`: Module perekam stream video dari kamera.
- `models/`: Kumpulan model terkompilasi RKNN (`LESGO11m`, `LESGOv8s`, dll).

---

### 4. Sensor Package (`main/src/sensor`)

**Core Components**:
- `rplidar.cpp`: Pengolahan data scan 2D RPLidar untuk rintangan.
- `livox.cpp`: Interface pemprosesan LiDAR 3D Livox.
- `remote.cpp`: Handler masukan transmiter remote control RC.
- `state.cpp`: Pengawas status telemetri hardware.

---

### 5. Monitoring Package (`main/src/monitoring`)

**Core Components**:
- `battery.cpp`: Monitoring tegangan dan daya baterai.
- `cpu.cpp`: Resource usage CPU dan temperatur sistem.
- `gps.cpp`: Monitoring status dan presisi koordinat GPS.
- `date.py`: Publisher waktu dan tanggal sistem.

---

### 6. Launcher Package (`main/src/launcher`)

**Configuration & Launch**:
- **config/**: File konfigurasi YAML untuk navigasi, sensor, persepsi, dan misi.
- **launch/**: Skrip peluncur utama (`mission/gps_compass4.launch.py`, `mission/gps_vision.launch.py`, `rosbridge.launch.py`, `mavros.launch.py`).
- **debug/**: Script debugging bash untuk pengujian manual.

---

### 7. Web UI Dashboard (`web_ui`)

**Dashboard Monitoring & Telemetri Web**:
- Aplikasi berbasis **Next.js & ROSBridge Web Socket** untuk pemantauan posisi kapal, status baterai, feed kamera, dan kontrol mode secara real-time dari web browser.

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
- **Detection Parameters**: Configurable confidence, IoU threshold, and image size
- **PID Parameters**: Vision navigation control gains
- **RC Control**: Throttle values and control ranges
- **Camera Settings**: Resolution and capture parameters

## Hardware Integration

### Supported Hardware
- **Flight Controller**: ArduPilot-compatible autopilot via MAVLink
- **Compute Platform**: Orange Pi 5 Pro with NPU acceleration (recommended)
- **LIDAR**: RPLidar A1/A2/A3 series
- **Camera**: USB camera for vision processing
- **RC Receiver**: Arduino-based RC input processing
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
- **Vision Processing**: 20 Hz with RKNN NPU acceleration
- **Sensor Monitoring**: 1 Hz for non-critical systems

### Resource Utilization
- **CPU**: Optimized for multi-core processing
- **NPU**: Full utilization of Orange Pi 5 Pro's 3 NPU cores
- **Memory**: Efficient frame buffering and processing
- **Network**: Minimal latency ROS 2 communication with QoS profiles

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
- **Advanced Machine Learning**: Adaptive navigation algorithms
- **Enhanced NPU Integration**: Additional AI acceleration features
- **Simulation Integration**: Gazebo-based testing environment

### Scalability Considerations
- **Modular Architecture**: Easy component addition/removal
- **Configuration Management**: Centralized parameter system with QoS profiles
- **Performance Optimization**: NPU acceleration and parallel processing
- **Hardware Flexibility**: Support for various compute platforms
- **Documentation**: Comprehensive developer documentation
