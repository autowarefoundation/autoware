cmake_minimum_required(VERSION 3.14)
project(emergency_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(emergency_handler
  src/emergency_handler/emergency_handler_node.cpp
  src/emergency_handler/emergency_handler_core.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
