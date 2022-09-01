cmake_minimum_required(VERSION 3.14)
project(ad_api_adaptors)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(initial_pose_adaptor
  src/initial_pose_adaptor.cpp
)

ament_auto_add_executable(routing_adaptor
  src/routing_adaptor.cpp
)

ament_auto_package(INSTALL_TO_SHARE config launch)
