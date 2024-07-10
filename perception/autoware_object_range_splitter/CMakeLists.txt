cmake_minimum_required(VERSION 3.14)
project(autoware_object_range_splitter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/object_range_splitter_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::object_range_splitter::ObjectRangeSplitterNode"
  EXECUTABLE object_range_splitter_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
