cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_multi_camera_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_multi_camera_fusion_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::MultiCameraFusion"
  EXECUTABLE traffic_light_multi_camera_fusion_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
