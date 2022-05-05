cmake_minimum_required(VERSION 3.14)
project(object_range_splitter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(object_range_splitter SHARED
  src/node.cpp
)

rclcpp_components_register_node(object_range_splitter
  PLUGIN "object_range_splitter::ObjectRangeSplitterNode"
  EXECUTABLE object_range_splitter_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
