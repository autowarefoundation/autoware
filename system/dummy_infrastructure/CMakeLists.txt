cmake_minimum_required(VERSION 3.14)
project(dummy_infrastructure)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(dummy_infrastructure_node_component SHARED
  src/dummy_infrastructure_node/dummy_infrastructure_node.cpp
)

rclcpp_components_register_node(dummy_infrastructure_node_component
  PLUGIN "dummy_infrastructure::DummyInfrastructureNode"
  EXECUTABLE dummy_infrastructure_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
