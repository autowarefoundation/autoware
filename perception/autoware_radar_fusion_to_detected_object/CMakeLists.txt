cmake_minimum_required(VERSION 3.5)
project(autoware_radar_fusion_to_detected_object)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/radar_fusion_to_detected_object.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::radar_fusion_to_detected_object::RadarObjectFusionToDetectedObjectNode"
  EXECUTABLE radar_fusion_to_detected_object_node
)

# Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
