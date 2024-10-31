cmake_minimum_required(VERSION 3.5)
project(uvs_tools)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGES(
  ament_cmake 
  rclcpp  
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  uvs_message
)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/uvs_tools
  ${CMAKE_SOURCE_DIR}/thirdparty/jsoncpp/include
  ${CMAKE_SOURCE_DIR}/thirdparty/tinyxml2
)

file (GLOB UVS_TOOLS_SRC_FILES
  "${CMAKE_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_SOURCE_DIR}/thirdparty/jsoncpp/src/lib_json/*.cpp"
  "${CMAKE_SOURCE_DIR}/thirdparty/tinyxml2/*.cpp"
)

BUILD_LIB(
  TARGET uvs_tools
  SOURCES ${UVS_TOOLS_SRC_FILES}
  DEPENDENCIES nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_message
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
