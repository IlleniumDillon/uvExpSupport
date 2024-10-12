cmake_minimum_required(VERSION 3.5)
project(uvs_manual)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGES(
  ament_cmake 
  rclcpp  
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  Boost
  Threads
  uvs_message
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/uvs_manual_joystick
  ${CMAKE_CURRENT_SOURCE_DIR}/include/uvs_manual_keyboard
)

file (GLOB UVS_MANUAL_JOYSTICK_SOURCE "src/uvs_manual_joystick/*.cpp")

BUILD_EXEC(
  TARGET uvs_manual_joystick
  SOURCES ${UVS_MANUAL_JOYSTICK_SOURCE}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_message Boost Threads
)

file (GLOB UVS_MANUAL_KEYBOARD_SOURCE "src/uvs_manual_keyboard/*.cpp")

BUILD_EXEC(
  TARGET uvs_manual_keyboard
  SOURCES ${UVS_MANUAL_KEYBOARD_SOURCE}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_message Boost Threads
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
