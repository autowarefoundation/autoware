cmake_minimum_required(VERSION 3.14)
project(pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

ament_auto_add_executable(pose_initializer
  src/pose_initializer_node.cpp
  src/pose_initializer_core.cpp
)
target_link_libraries(pose_initializer ${PCL_LIBRARIES})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
