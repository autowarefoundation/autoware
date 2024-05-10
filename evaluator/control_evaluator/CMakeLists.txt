cmake_minimum_required(VERSION 3.14)
project(control_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/${PROJECT_NAME}_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "control_diagnostics::controlEvaluatorNode"
  EXECUTABLE ${PROJECT_NAME}
)


ament_auto_package(
  INSTALL_TO_SHARE
    param
    launch
)
