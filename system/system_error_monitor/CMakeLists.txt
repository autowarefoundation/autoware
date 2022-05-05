cmake_minimum_required(VERSION 3.14)
project(system_error_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/system_error_monitor_node.cpp
  src/system_error_monitor_core.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
