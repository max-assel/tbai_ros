cmake_minimum_required(VERSION 3.16)

project(tbai_ros_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Declare the message files to be built
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/JointCommand.msg"
        "msg/JointCommandArray.msg"
        "msg/RbdState.msg"
        "msg/RobotState.msg"
        "msg/EstimatedState.msg"
    DEPENDENCIES 
        # std_msgs 
        builtin_interfaces
)

# Install message and service files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()