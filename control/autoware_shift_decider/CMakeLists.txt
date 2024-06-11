cmake_minimum_required(VERSION 3.14)
project(autoware_shift_decider)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/${PROJECT_NAME}.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::shift_decider::ShiftDecider"
  EXECUTABLE ${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
