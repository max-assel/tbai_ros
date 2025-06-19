cmake_minimum_required(VERSION 3.19.0)
project(tbai CXX)

include(ExternalProject)
if(DEFINED TBAI_DEVELOP_PATH)
  message(STATUS "Using local tbai2 path: ${TBAI_DEVELOP_PATH}")
  ExternalProject_Add(
    tbai2
    SOURCE_DIR ${TBAI_DEVELOP_PATH}
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    UPDATE_COMMAND "echo"
  )
else()
  message(STATUS "Using remote tbai2")
  ExternalProject_Add(
    tbai2
    GIT_REPOSITORY git@github.com:lnotspotl/tbai2.git
    GIT_TAG main
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    UPDATE_COMMAND ""
  )
endif()
find_package(catkin REQUIRED)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  LIBRARIES tbai_core tbai_bob
)