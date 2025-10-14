cmake_minimum_required(VERSION 3.15)
project(tbai_ros_core)

## C++17 is a requirement
add_compile_options(-std=c++17)

# ament_cmake dependencies
set(dependencies
  tbai_ros_msgs
  std_msgs
  rclcpp
  tbai
)

# Find ROS2 build system
find_package(ament_cmake REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Dependencies
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

# find_package(catkin REQUIRED COMPONENTS
#   tf
#   tbai_ros_msgs
#   std_msgs
#   roscpp
#   roslib
#   tbai
# )

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES  ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp roslib tbai_ros_msgs
# )

# Build
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Subscribers.cpp
)
# Declare dependencies for build/export
ament_target_dependencies(${PROJECT_NAME}
	${dependencies}
)
target_link_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()