cmake_minimum_required(VERSION 3.14)
project(yabloc_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(yabloc_monitor
  src/yabloc_monitor_node.cpp
  src/yabloc_monitor_core.cpp
  src/availability_module.cpp
)
ament_target_dependencies(yabloc_monitor)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
