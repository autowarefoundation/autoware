cmake_minimum_required(VERSION 3.14)
project(mrm_comfortable_stop_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(mrm_comfortable_stop_operator_component SHARED
  src/mrm_comfortable_stop_operator/mrm_comfortable_stop_operator_core.cpp
)

rclcpp_components_register_node(mrm_comfortable_stop_operator_component
  PLUGIN "mrm_comfortable_stop_operator::MrmComfortableStopOperator"
  EXECUTABLE mrm_comfortable_stop_operator)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
