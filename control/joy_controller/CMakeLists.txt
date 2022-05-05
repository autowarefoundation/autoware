cmake_minimum_required(VERSION 3.14)
project(joy_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(joy_controller_node SHARED
  src/joy_controller/joy_controller_node.cpp
)

rclcpp_components_register_node(joy_controller_node
  PLUGIN "joy_controller::AutowareJoyControllerNode"
  EXECUTABLE joy_controller
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
