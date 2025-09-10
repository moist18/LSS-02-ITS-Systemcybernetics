cmake_minimum_required(VERSION 3.5)
project(monitoring)

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
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/ubuntu/ASV-2025/libs/install")
find_package(mavros_msgs REQUIRED)

# C++ executables
add_executable(battery ${PROJECT_NAME}/battery.cpp)
ament_target_dependencies(battery
  rclcpp
  std_msgs
  mavros_msgs
)

add_executable(cpu ${PROJECT_NAME}/cpu.cpp)
ament_target_dependencies(cpu
  rclcpp
  std_msgs
)

add_executable(gps ${PROJECT_NAME}/gps.cpp)
ament_target_dependencies(gps
  rclcpp
  std_msgs
  mavros_msgs
)

add_executable(date ${PROJECT_NAME}/date.cpp)
ament_target_dependencies(date
  rclcpp
  std_msgs
)

# Install C++ executables
install(TARGETS
  battery
  cpu
  gps
  date
  DESTINATION lib/${PROJECT_NAME}
)

# Python package installation
ament_python_install_package(${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()