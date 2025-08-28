cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_g1_rl)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  tf
  urdf
  kdl_parser
  robot_state_publisher
  tbai_ros_msgs
  tbai_ros_core
  tbai_ros_static
  tbai_ros_gridmap
  tbai_ros_reference
  geometry_msgs
  tbai
  visualization_msgs
)