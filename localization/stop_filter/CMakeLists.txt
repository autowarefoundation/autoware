cmake_minimum_required(VERSION 3.14)
project(stop_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(stop_filter
  src/stop_filter_node.cpp
  src/stop_filter.cpp
)
ament_target_dependencies(stop_filter)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
