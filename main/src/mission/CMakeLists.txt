cmake_minimum_required(VERSION 3.5)
project(mission)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# MAVROS packages
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/ubuntu/ASV-2025/libs/install")
find_package(mavros_msgs REQUIRED)

# C++ executables
add_executable(path_planning ${PROJECT_NAME}/path_planning.cpp)
ament_target_dependencies(path_planning
  rclcpp
  std_msgs
  mavros_msgs
)

# Install C++ executables
install(TARGETS
  path_planning
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()