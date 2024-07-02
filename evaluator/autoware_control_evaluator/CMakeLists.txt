cmake_minimum_required(VERSION 3.14)
project(autoware_control_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)

ament_auto_add_library(control_evaluator_node SHARED
  src/control_evaluator_node.cpp
  src/metrics/deviation_metrics.cpp
)

rclcpp_components_register_node(control_evaluator_node
  PLUGIN "control_diagnostics::ControlEvaluatorNode"
  EXECUTABLE control_evaluator
)


ament_auto_package(
  INSTALL_TO_SHARE
    param
    launch
)
