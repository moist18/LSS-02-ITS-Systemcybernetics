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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Set path to find mavros_msgs from libs workspace
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/arielsulton/ASV-2025/libs/install")
find_package(mavros_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(pose_planner ${PROJECT_NAME}/pose_planner.cpp)
ament_target_dependencies(pose_planner 
  rclcpp 
  std_msgs
  mavros_msgs
)
target_link_libraries(pose_planner yaml-cpp)

install(TARGETS
  pose_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()