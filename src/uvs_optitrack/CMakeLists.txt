cmake_minimum_required(VERSION 3.5)
project(uvs_optitrack)

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
  ${CMAKE_CURRENT_SOURCE_DIR}/NatNet
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/uvs_optitrack
)

file (GLOB UVS_OPTITRACK_SOURCE "src/*.cpp")

BUILD_EXEC(
  TARGET uvs_optitrack
  SOURCES ${UVS_OPTITRACK_SOURCE}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_message Boost Threads
)

target_link_directories(uvs_optitrack PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/NatNet)
target_link_libraries(uvs_optitrack NatNet)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/NatNet/ 
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
