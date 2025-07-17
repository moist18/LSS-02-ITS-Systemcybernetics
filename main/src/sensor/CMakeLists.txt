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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Set path to find mavros_msgs from libs workspace
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/arielsulton/ASV-2025/libs/install")
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

# Include directories for serial library
include_directories(/home/arielsulton/ASV-2025/libs/install/serial/include)

add_executable(remote ${PROJECT_NAME}/remote.cpp)
ament_target_dependencies(remote
  rclcpp
  std_msgs
  mavros_msgs
  sensor_msgs
)
target_link_libraries(remote /home/arielsulton/ASV-2025/libs/build/serial/libserial.a)

add_executable(rplidar ${PROJECT_NAME}/rplidar.cpp)
ament_target_dependencies(rplidar
  rclcpp
  std_msgs
  mavros_msgs
)

add_executable(serial_node ${PROJECT_NAME}/serial.cpp)
ament_target_dependencies(serial_node
  rclcpp
  std_msgs
  mavros_msgs
)
target_link_libraries(serial_node /home/arielsulton/ASV-2025/libs/build/serial/libserial.a)

add_executable(state ${PROJECT_NAME}/state.cpp)
ament_target_dependencies(state
  rclcpp
  std_msgs
  mavros_msgs
)

add_executable(compass ${PROJECT_NAME}/compass.cpp)
ament_target_dependencies(compass
  rclcpp
  std_msgs
)

add_executable(gps ${PROJECT_NAME}/gps.cpp)
ament_target_dependencies(gps
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(emergency ${PROJECT_NAME}/emergency.cpp)
ament_target_dependencies(emergency
  rclcpp
  std_msgs
)

install(TARGETS
  remote
  rplidar
  serial_node
  state
  compass
  gps
  emergency
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()