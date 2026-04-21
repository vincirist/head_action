cmake_minimum_required(VERSION 3.14.4) # Humble requires 3.14+
project(head_action)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. Find buildtool
find_package(ament_cmake REQUIRED)

# 2. Find dependencies (Replaced roscpp with rclcpp and added tf2)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED) # The ROS 2 Humble shim for KDL

# 3. Find KDL (usually exported via orocos_kdl_vendor)
find_package(orocos_kdl REQUIRED)

# 4. Executable setup
add_executable(head_action src/point_frame.cpp)

# 5. Link dependencies (Use ament_target_dependencies for simplicity)
# This macro handles both include directories and linking for you.
ament_target_dependencies(head_action
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  kdl_parser
  orocos_kdl
)

# 6. Install rules (Standard for ROS 2)
install(TARGETS head_action
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# 7. Export and close
ament_package()