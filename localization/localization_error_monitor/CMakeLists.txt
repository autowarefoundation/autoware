cmake_minimum_required(VERSION 3.14)
project(localization_error_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(localization_error_monitor
  src/main.cpp
  src/node.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
