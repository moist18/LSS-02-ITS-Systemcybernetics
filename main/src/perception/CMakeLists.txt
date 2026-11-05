cmake_minimum_required(VERSION 3.5)
project(perception)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ROS2 packages
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# MAVROS packages
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/orin/TEKNOFEST-2026/USV-2026/main/libs/install")
find_package(mavros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)


# Python package installation
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/recorder.py
  ${PROJECT_NAME}/vision.py
  ${PROJECT_NAME}/vision_depth.py
  ${PROJECT_NAME}/visionDepth.py
  ${PROJECT_NAME}/vision_tekno.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install models alongside Python executables (lib/) so ROOT/models works at runtime
install(DIRECTORY
  ${PROJECT_NAME}/models/
  DESTINATION lib/${PROJECT_NAME}/models
)

# Also install to share/ for ROS convention
install(DIRECTORY
  ${PROJECT_NAME}/models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY
  ${PROJECT_NAME}/utils/
  DESTINATION share/${PROJECT_NAME}/utils
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()