cmake_minimum_required(VERSION 3.14)
project(automatic_pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(automatic_pose_initializer
  src/automatic_pose_initializer.cpp
)

ament_auto_package(INSTALL_TO_SHARE launch)
