cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_mpc_blind)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tbai_ros_core
    tbai_ros_mpc
    tbai_ros_static
    tbai
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES  
 CATKIN_DEPENDS  
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(simple
    src/simple.cpp
)
target_link_libraries(simple
    ${catkin_LIBRARIES} yaml-cpp
)

add_executable(simple_deploy_go2
    src/simple_deploy_go2.cpp
)
target_link_libraries(simple_deploy_go2
    ${catkin_LIBRARIES} yaml-cpp
)