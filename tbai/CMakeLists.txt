cmake_minimum_required(VERSION 3.19.0)
project(tbai CXX)

find_package(catkin REQUIRED)
find_package(tbai REQUIRED)
find_package(Eigen3 REQUIRED)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES tbai_core tbai_bob tbai_np3o tbai_static tbai_estim tbai_deploy_go2 unitree_sdk2
)