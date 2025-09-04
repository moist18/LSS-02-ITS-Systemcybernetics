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
add_executable(gps_rtk_planner ${PROJECT_NAME}/gps_rtk_planner.cpp)
ament_target_dependencies(gps_rtk_planner 
  rclcpp 
  std_msgs
  mavros_msgs
)

add_executable(gps_vision_planner ${PROJECT_NAME}/gps_vision_planner.cpp)
ament_target_dependencies(gps_vision_planner 
  rclcpp 
  std_msgs
  mavros_msgs
)

# Install C++ executables
install(TARGETS
  gps_rtk_planner
  gps_vision_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()