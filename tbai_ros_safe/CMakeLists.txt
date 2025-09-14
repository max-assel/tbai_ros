cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_safe)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  visualization_msgs
  tbai_ros_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs visualization_msgs tbai_ros_msgs
)

catkin_install_python(PROGRAMS
  src/tbai_safe_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/tbai_safe_node_mppi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
