cmake_minimum_required(VERSION 3.5)
project(uvs_embmaster)

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
  ${CMAKE_CURRENT_SOURCE_DIR}/libusb
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/uvs_embmaster
  ${CMAKE_CURRENT_SOURCE_DIR}/include/protocol
  ${CMAKE_CURRENT_SOURCE_DIR}/ThreadManager
)

file (GLOB UVS_EMBMASTER_SOURCE "src/*.cpp" "src/*.c")

BUILD_EXEC(
  TARGET uvs_embmaster
  SOURCES ${UVS_EMBMASTER_SOURCE}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_message Boost Threads
)

target_link_directories(uvs_embmaster PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libusb)
target_link_libraries(uvs_embmaster usb-1.0)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/libusb/ 
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
