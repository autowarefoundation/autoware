cmake_minimum_required(VERSION 3.14)
project(dummy_diag_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(dummy_diag_publisher SHARED
  src/dummy_diag_publisher_node/dummy_diag_publisher_node.cpp
)

rclcpp_components_register_node(dummy_diag_publisher
  PLUGIN "DummyDiagPublisherNode"
  EXECUTABLE dummy_diag_publisher_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
