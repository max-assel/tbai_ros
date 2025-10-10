cmake_minimum_required(VERSION 3.5)
project(tbai_ros_utils)

find_package(ament_cmake REQUIRED)

catkin_package()

install(TARGETS 
  src/virtual_joystick.py
    DESTINATION lib/${PROJECT_NAME}
)
