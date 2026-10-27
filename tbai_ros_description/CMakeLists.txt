cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_description)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY back_platform/meshes back_platform/dae back_platform/urdf back_platform/xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/back_platform
)
