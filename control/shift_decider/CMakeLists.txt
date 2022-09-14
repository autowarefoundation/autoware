cmake_minimum_required(VERSION 3.14)
project(shift_decider)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(shift_decider_node SHARED
  src/shift_decider.cpp
)

rclcpp_components_register_node(shift_decider_node
  PLUGIN "ShiftDecider"
  EXECUTABLE shift_decider
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
