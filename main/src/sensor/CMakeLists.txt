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
set(CMAKE_PREFIX_PATH
  "${CMAKE_PREFIX_PATH}"
  "/home/ubuntu/ASV-2025/libs/install"
  "/home/aryaputra/Desktop/ssh/LSS_02/libs/install"
  "/home/orin/LSS_02/libs/install"
  "/home/aryaputra/Desktop/ssh/TEKNOFEST-2026/USV-2026/install/serial"
)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Serial packages — auto-detect serial cmake config across known machine paths
# Use CACHE PATH FORCE to override any stale serial_DIR-NOTFOUND in CMake cache
set(_SERIAL_CMAKE_CANDIDATES
  "/home/ubuntu/ASV-2025/libs/install/serial/share/serial/cmake"
  "/home/aryaputra/Desktop/ssh/LSS_02/libs/install/serial/share/serial/cmake"
  "/home/orin/LSS_02/libs/install/serial/share/serial/cmake"
  "/home/aryaputra/Desktop/ssh/TEKNOFEST-2026/USV-2026/install/serial/share/serial/cmake"
)
foreach(_candidate ${_SERIAL_CMAKE_CANDIDATES})
  if(EXISTS "${_candidate}/serialConfig.cmake" AND NOT serial_FOUND)
    set(serial_DIR "${_candidate}" CACHE PATH "" FORCE)
  endif()
endforeach()
if(NOT serial_DIR OR serial_DIR STREQUAL "serial_DIR-NOTFOUND")
  message(FATAL_ERROR "serial cmake config not found! Build libs first with: cd libs && colcon build")
endif()
find_package(serial REQUIRED)

# Serial include directories
set(_SERIAL_INCLUDE_CANDIDATES
  "/home/ubuntu/ASV-2025/libs/install/serial/include"
  "/home/aryaputra/Desktop/ssh/LSS_02/libs/install/serial/include"
  "/home/orin/LSS_02/libs/install/serial/include"
  "/home/aryaputra/Desktop/ssh/TEKNOFEST-2026/USV-2026/install/serial/include"
)
foreach(_inc ${_SERIAL_INCLUDE_CANDIDATES})
  if(EXISTS "${_inc}")
    include_directories("${_inc}")
  endif()
endforeach()

# C++ executables
add_executable(remote ${PROJECT_NAME}/remote.cpp)
ament_target_dependencies(remote
  rclcpp
  std_msgs
  mavros_msgs
  sensor_msgs
)
# Auto-detect whichever libserial.a is available
set(_SERIAL_LIB_CANDIDATES
  "/home/ubuntu/ASV-2025/libs/install/serial/lib/libserial.a"
  "/home/aryaputra/Desktop/ssh/LSS_02/libs/install/serial/lib/libserial.a"
  "/home/orin/LSS_02/libs/install/serial/lib/libserial.a"
  "/home/aryaputra/Desktop/ssh/TEKNOFEST-2026/USV-2026/install/serial/lib/libserial.a"
)
foreach(_lib ${_SERIAL_LIB_CANDIDATES})
  if(EXISTS "${_lib}" AND NOT SERIAL_LIB)
    set(SERIAL_LIB "${_lib}")
  endif()
endforeach()
if(NOT SERIAL_LIB)
  message(FATAL_ERROR "libserial.a not found! Build libs first with: cd libs && colcon build")
endif()
target_link_libraries(remote ${SERIAL_LIB})

add_executable(rplidar ${PROJECT_NAME}/rplidar.cpp)
ament_target_dependencies(rplidar
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(livox ${PROJECT_NAME}/livox.cpp)
ament_target_dependencies(livox
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(state ${PROJECT_NAME}/state.cpp)
ament_target_dependencies(state
  rclcpp
  std_msgs
  mavros_msgs
)

# Install C++ executables
install(TARGETS
  remote
  rplidar
  livox
  state
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()