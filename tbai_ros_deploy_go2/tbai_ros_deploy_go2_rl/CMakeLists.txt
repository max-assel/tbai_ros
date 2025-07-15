cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_deploy_go2_rl)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    tbai
    tbai_ros_core
    tbai_ros_np3o
    tbai_ros_bob
    tbai_ros_static
    tbai_ros_reference
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS tbai_ros_np3o tbai_ros_bob tbai_ros_static tbai_ros_reference
)


add_executable(deploy_go2_rl
    src/deploy_go2_rl.cpp
)
target_link_libraries(deploy_go2_rl
    ${catkin_LIBRARIES} yaml-cpp
)
target_include_directories(deploy_go2_rl 
    PRIVATE
    "${catkin_INCLUDE_DIRS}" include
)