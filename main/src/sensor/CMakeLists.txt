cmake_minimum_required(VERSION 3.5)
project(sensor)

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
find_package(sensor_msgs REQUIRED)

# Serial packages
find_package(serial REQUIRED)
include_directories(/home/ubuntu/ASV-2025/libs/install/serial/include)

# C++ executables
add_executable(arduino ${PROJECT_NAME}/arduino.cpp)
ament_target_dependencies(arduino
  rclcpp
  std_msgs
  mavros_msgs
  sensor_msgs
)
target_link_libraries(arduino /home/ubuntu/ASV-2025/libs/build/serial/libserial.a)

add_executable(state ${PROJECT_NAME}/state.cpp)
ament_target_dependencies(state
  rclcpp
  std_msgs
  mavros_msgs
)

# Install C++ executables
install(TARGETS
  arduino
  state
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()