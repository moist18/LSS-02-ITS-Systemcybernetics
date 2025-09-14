cmake_minimum_required(VERSION 3.5)
project(navigation)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# C++ executables
add_executable(gps_rtk_nav ${PROJECT_NAME}/gps_rtk_nav.cpp)
ament_target_dependencies(gps_rtk_nav
  rclcpp
  std_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  geographic_msgs
  sensor_msgs
  nav_msgs
)

add_executable(gps_vision_nav ${PROJECT_NAME}/gps_vision_nav.cpp)
ament_target_dependencies(gps_vision_nav
  rclcpp
  std_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  geographic_msgs
  sensor_msgs
  nav_msgs
)

add_executable(vision_nav ${PROJECT_NAME}/vision_nav.cpp)
ament_target_dependencies(vision_nav
  rclcpp
  std_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  geographic_msgs
  sensor_msgs
  nav_msgs
)

# Install C++ executables
install(TARGETS
  gps_rtk_nav
  gps_vision_nav
  vision_nav
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()