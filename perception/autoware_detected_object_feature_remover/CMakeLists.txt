cmake_minimum_required(VERSION 3.14)
project(autoware_detected_object_feature_remover)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detected_object_feature_remover_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::detected_object_feature_remover::DetectedObjectFeatureRemover"
  EXECUTABLE detected_object_feature_remover_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
