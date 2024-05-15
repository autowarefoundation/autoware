cmake_minimum_required(VERSION 3.14)
project(system_error_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/system_error_monitor_core.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "AutowareErrorMonitor"
  EXECUTABLE ${PROJECT_NAME}_node)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
