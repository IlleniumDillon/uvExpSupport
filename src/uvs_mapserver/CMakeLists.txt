cmake_minimum_required(VERSION 3.5)
project(uvs_mapserver)

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
  uvs_tools
  uvs_message
)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/uvs_mapserver

)

file (GLOB UVS_MAPSERVER_SRC_FILES
  "${CMAKE_SOURCE_DIR}/src/uvs_mapserver.cpp"
  "${CMAKE_SOURCE_DIR}/src/uvs_mapserver_main.cpp"

)

BUILD_EXEC(
  TARGET uvs_mapserver
  SOURCES ${UVS_MAPSERVER_SRC_FILES}
  DEPENDENCIES rclcpp gazebo_dev gazebo_msgs gazebo_ros geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_tools uvs_message
)

BUILD_EXEC(
  TARGET uvs_mapserver_test
  SOURCES ${CMAKE_SOURCE_DIR}/src/uvs_mapserver_test.cpp
  DEPENDENCIES rclcpp gazebo_dev gazebo_msgs gazebo_ros geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_message uvs_tools
)

set(WORLDXACROFILE ${CMAKE_SOURCE_DIR}/world/jxl3028.world.xacro)
GEN_XACRO(${WORLDXACROFILE})

install(
  DIRECTORY models world config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
