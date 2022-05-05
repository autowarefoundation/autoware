cmake_minimum_required(VERSION 3.14)
project(external_velocity_limit_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(external_velocity_limit_selector_node SHARED
  src/external_velocity_limit_selector_node.cpp
)

rclcpp_components_register_node(external_velocity_limit_selector_node
  PLUGIN "ExternalVelocityLimitSelectorNode"
  EXECUTABLE external_velocity_limit_selector
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
