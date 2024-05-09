cmake_minimum_required(VERSION 3.14)
project(autoware_pose_covariance_modifier)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_covariance_modifier.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose_covariance_modifier::PoseCovarianceModifierNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
