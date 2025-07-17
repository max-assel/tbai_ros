cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_mpc)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tbai_ros_core
    tbai_ros_msgs
    ocs2_core
    ocs2_mpc
    ocs2_msgs
    ocs2_ros_interfaces
    ocs2_anymal_commands
    grid_map_core
    grid_map_ros
    tbai_ros_static
    tbai_ros_reference

    roscpp
    std_msgs
    qpoases_catkin
    ocs2_mpc
    ocs2_core
    ocs2_anymal_mpc
    ocs2_ros_interfaces
    ocs2_pinocchio_interface
    ocs2_quadruped_interface
    ocs2_switched_model_interface
    tbai
)

find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tbai_ros_mpc
 CATKIN_DEPENDS ocs2_anymal_mpc tbai_ros_reference
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${pinocchio_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/reference/ReferenceTrajectoryGenerator.cpp
    src/wbc/Task.cpp
    src/wbc/SqpSolver.cpp
    src/wbc/SqpWbc.cpp
    src/wbc/HqpSolver.cpp
    src/wbc/HqpWbc.cpp
    src/wbc/Factory.cpp
    src/wbc/WbcBase.cpp
    src/utils/LoadConfig.cpp
    src/MpcController.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${catkin_LIBRARIES} spdlog::spdlog
    ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(anymal_mpc_node
  src/MpcNode.cpp
)
target_link_libraries(anymal_mpc_node
  ${catkin_LIBRARIES} yaml-cpp spdlog::spdlog
)
target_compile_options(anymal_mpc_node PRIVATE ${OCS2_CXX_FLAGS})

add_executable(simple_blind
    src/simple_blind.cpp
)
target_link_libraries(simple_blind
    ${catkin_LIBRARIES} yaml-cpp ${PROJECT_NAME}
)

add_executable(simple_perceptive
    src/simple_perceptive.cpp
)
target_link_libraries(simple_perceptive
    ${catkin_LIBRARIES} yaml-cpp ${PROJECT_NAME}
)