cmake_minimum_required(VERSION 3.14)
project(autoware_joy_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_joy_controller_node SHARED
  DIRECTORY src
)

rclcpp_components_register_node(autoware_joy_controller_node
  PLUGIN "autoware::joy_controller::AutowareJoyControllerNode"
  EXECUTABLE autoware_joy_controller
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
