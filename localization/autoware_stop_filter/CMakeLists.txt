cmake_minimum_required(VERSION 3.14)
project(autoware_stop_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/stop_filter.cpp
  src/stop_filter.hpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::stop_filter::StopFilter"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
