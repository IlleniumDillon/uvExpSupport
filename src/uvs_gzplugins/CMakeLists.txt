cmake_minimum_required(VERSION 3.5)
project(uvs_gzplugins)

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
  uvs_tools
  Qt5Widgets
  Qt5Core
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/uvs_gzplugins
  ${Qt5Core_INCLUDE_DIRS}
)

BUILD_GAZEBO_PLUGIN(
  TARGET gz_ros_simchassis 
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/gz_ros_simchassis.cpp 
  DEPENDENCIES rclcpp gazebo_dev gazebo_msgs gazebo_ros geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_message uvs_tools
)
BUILD_GAZEBO_PLUGIN(
  TARGET gz_ros_simoptitrack 
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/gz_ros_simoptitrack.cpp
  DEPENDENCIES rclcpp gazebo_dev gazebo_msgs gazebo_ros geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_message uvs_tools
)


if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/gz_ros_simgui.hpp)

BUILD_GAZEBO_PLUGIN(
  TARGET gz_ros_simgui
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/gz_ros_simgui.cpp ${MOC_FILES}
  DEPENDENCIES rclcpp gazebo_dev gazebo_msgs gazebo_ros geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_message uvs_tools Qt5Core Qt5Widgets
)

set(WORLDXACROFILE ${CMAKE_SOURCE_DIR}/worlds/uvs_gazebo.world.xacro)
GEN_XACRO(${WORLDXACROFILE})

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
