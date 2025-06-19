cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_rl_perceptive)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  tbai_ros_core
  tbai_ros_static
  tbai_ros_rl
  tbai
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS 
  DEPENDS 
)

add_executable(simple
    src/simple.cpp
)
target_link_libraries(simple
    ${catkin_LIBRARIES} yaml-cpp
)