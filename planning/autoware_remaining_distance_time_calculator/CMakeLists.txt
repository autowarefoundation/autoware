cmake_minimum_required(VERSION 3.14)
project(autoware_remaining_distance_time_calculator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/remaining_distance_time_calculator_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::remaining_distance_time_calculator::RemainingDistanceTimeCalculatorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)


ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
