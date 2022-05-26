cmake_minimum_required(VERSION 3.14)
project(traffic_light_map_based_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(traffic_light_map_based_detector SHARED
  src/node.cpp
)

rclcpp_components_register_node(traffic_light_map_based_detector
  PLUGIN "traffic_light::MapBasedDetector"
  EXECUTABLE traffic_light_map_based_detector_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
