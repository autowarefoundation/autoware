cmake_minimum_required(VERSION 3.14)
project(autoware_path_distance_calculator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/path_distance_calculator.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::path_distance_calculator::PathDistanceCalculator"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE launch)
