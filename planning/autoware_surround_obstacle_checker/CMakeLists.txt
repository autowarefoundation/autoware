cmake_minimum_required(VERSION 3.14)
project(autoware_surround_obstacle_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

generate_parameter_library(surround_obstacle_checker_node_parameters
  param/surround_obstacle_checker_node_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/debug_marker.cpp
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  surround_obstacle_checker_node_parameters
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::surround_obstacle_checker::SurroundObstacleCheckerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
