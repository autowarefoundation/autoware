cmake_minimum_required(VERSION 3.14)
project(autoware_processing_time_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(rclcpp REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/processing_time_checker.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::processing_time_checker::ProcessingTimeChecker"
  EXECUTABLE processing_time_checker_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
