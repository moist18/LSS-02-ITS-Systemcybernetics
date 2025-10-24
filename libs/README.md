# Third-Party Libraries Installation

This directory contains ROS 2 Humble third-party dependencies for the ASV-2025 project.

## Prerequisites Installation

Install required build tools and dependencies:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool python3-rosinstall-generator python3-osrf-pycommon
```

## Python Dependencies

Install compatible NumPy and OpenCV versions for cv_bridge:

```bash
pip install 'numpy<2' 'opencv-python==4.9.0.80'
```

**Why these versions?**
- `numpy<2`: cv_bridge (compiled with pybind11 2.9.1) requires NumPy 1.x
- `opencv-python==4.9.0.80`: Latest version compatible with NumPy 1.x
- NumPy 1.26.4 + OpenCV 4.9.0 is the tested stable combination for ROS 2 Humble

## Clone Dependencies

Navigate to the source directory:

```bash
cd ./libs/src/
```

### MAVROS Installation

Clone MAVROS (MAVLink to ROS bridge):

```bash
git clone -b ros2 https://github.com/mavlink/mavros.git
```

### ROS 2 Humble Libraries

Clone additional ROS 2 libraries:

```bash
git clone https://github.com/RoverRobotics-forks/serial-ros2.git
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
git clone -b humble https://github.com/ros-perception/vision_opencv.git
```

## MAVLink Installation

Generate and import MAVLink dependencies:

```bash
cd ..
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
vcs import src < /tmp/mavlink.repos
rosdep install --from-paths src --ignore-src -y
```

## Git Submodule Cleanup

Remove Git metadata from third-party packages:

```bash
cd src/
find ./ -type d -name ".git" -exec rm -rf {} +
find ./ -type f -name ".gitmodules" -exec rm -f {} +
find ./ -type f -name ".gitignore" -exec rm -f {} +
```

## Build Libraries

Build all libraries with colcon:

```bash
cd ..
colcon build
source ./install/setup.bash
```

## GeographicLib Datasets

Download and install GeographicLib datasets for MAVROS:

```bash
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm ./install_geographiclib_datasets.sh
```

## Commit Changes

Add and commit the built libraries:

```bash
cd ..
git add .
git commit -m "git commit"
git push -u origin humble
```

---

**Note**: Make sure to source the setup file in your `.bashrc` or current terminal session before using these libraries:

```bash
source /home/ubuntu/ASV-2025/libs/install/setup.bash
```