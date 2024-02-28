cmake_minimum_required(VERSION 3.14)
project(mrm_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(mrm_handler
  src/mrm_handler/mrm_handler_node.cpp
  src/mrm_handler/mrm_handler_core.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
