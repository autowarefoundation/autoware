cmake_minimum_required(VERSION 3.14)
project(component_interface_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/service_log_checker.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ServiceLogChecker"
  EXECUTABLE service_log_checker_node
)

ament_auto_package(INSTALL_TO_SHARE launch)
