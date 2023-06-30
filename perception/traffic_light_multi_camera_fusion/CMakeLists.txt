cmake_minimum_required(VERSION 3.14)
project(traffic_light_multi_camera_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_add_library(traffic_light_multi_camera_fusion SHARED
  src/node.cpp
)

rclcpp_components_register_node(traffic_light_multi_camera_fusion
  PLUGIN "traffic_light::MultiCameraFusion"
  EXECUTABLE traffic_light_multi_camera_fusion_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
