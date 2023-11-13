cmake_minimum_required(VERSION 3.14)
project(geo_pose_projector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(geo_pose_projector
  src/geo_pose_projector_node.cpp
  src/geo_pose_projector.cpp
)
ament_target_dependencies(geo_pose_projector)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
