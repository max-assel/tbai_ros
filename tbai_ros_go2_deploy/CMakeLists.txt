cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_go2_deploy)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    tbai
    tbai_ros_core
    tbai_ros_np3o
    tbai_ros_mpc
    tbai_ros_bob
    tbai_ros_static
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS tbai_ros_np3o tbai_ros_mpc tbai_ros_bob tbai_ros_static
)

add_executable(deploy_go2_static
    src/deploy_go2_static.cpp
)
target_link_libraries(deploy_go2_static
    ${catkin_LIBRARIES} yaml-cpp
)
target_include_directories(deploy_go2_static
    PRIVATE
    "${catkin_INCLUDE_DIRS}"
)

add_executable(deploy_go2_np3o
    src/deploy_go2_np3o.cpp
)
target_link_libraries(deploy_go2_np3o
    ${catkin_LIBRARIES} yaml-cpp
)
target_include_directories(deploy_go2_np3o 
    PRIVATE
    "${catkin_INCLUDE_DIRS}"
)


add_executable(deploy_go2_bob_blind
    src/deploy_go2_bob_blind.cpp
)
target_link_libraries(deploy_go2_bob_blind
    ${catkin_LIBRARIES} yaml-cpp
)
target_include_directories(deploy_go2_bob_blind
    PRIVATE
    "${catkin_INCLUDE_DIRS}"
)

add_executable(deploy_go2_mpc_blind
    src/deploy_go2_mpc_blind.cpp
)
target_link_libraries(deploy_go2_mpc_blind
    ${catkin_LIBRARIES} yaml-cpp
)
target_include_directories(deploy_go2_mpc_blind
    PRIVATE
    "${catkin_INCLUDE_DIRS}"
)