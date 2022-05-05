cmake_minimum_required(VERSION 3.14)
project(external_cmd_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(external_cmd_converter SHARED
  src/node.cpp
)

rclcpp_components_register_node(external_cmd_converter
  PLUGIN "external_cmd_converter::ExternalCmdConverterNode"
  EXECUTABLE external_cmd_converter_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
