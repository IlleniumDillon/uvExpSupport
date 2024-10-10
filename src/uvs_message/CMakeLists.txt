cmake_minimum_required(VERSION 3.5)
project(uvs_message)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGES(
  ament_cmake
  rclcpp
  geometry_msgs
  rosidl_default_generators
  nav_msgs
  OpenCV
  tf2
  tf2_ros
  tf2_geometry_msgs
)

file(GLOB EMB_MESSAGES_SOURCE "msg/*.msg" "srv/*.srv" "action/*.action")
GEN_MESSAGE(MESSAGES ${EMB_MESSAGES_SOURCE} DEPENDENCIES geometry_msgs nav_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
