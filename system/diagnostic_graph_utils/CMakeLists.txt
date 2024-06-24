cmake_minimum_required(VERSION 3.14)
project(diagnostic_graph_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lib/graph.cpp
  src/lib/subscription.cpp
)

ament_auto_add_library(${PROJECT_NAME}_tools SHARED
  src/node/dump.cpp
  src/node/converter.cpp
  src/node/logging.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "diagnostic_graph_utils::DumpNode"
  EXECUTABLE dump_node
)

rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "diagnostic_graph_utils::ConverterNode"
  EXECUTABLE converter_node
)

rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "diagnostic_graph_utils::LoggingNode"
  EXECUTABLE logging_node
)

ament_auto_package(INSTALL_TO_SHARE launch)
