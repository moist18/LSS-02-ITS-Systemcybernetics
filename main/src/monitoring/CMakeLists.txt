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

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Set path to find mavros_msgs from libs workspace
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/arielsulton/ASV-2025/libs/install")
find_package(mavros_msgs REQUIRED)

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

install(TARGETS
  battery
  cpu
  gps
  date
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/gpu.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()