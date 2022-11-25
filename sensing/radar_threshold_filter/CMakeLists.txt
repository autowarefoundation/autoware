cmake_minimum_required(VERSION 3.5)
project(radar_threshold_filter)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

## Targets
ament_auto_add_library(radar_threshold_filter_node_component SHARED
  src/radar_threshold_filter_node/radar_threshold_filter_node.cpp
)

rclcpp_components_register_node(radar_threshold_filter_node_component
  PLUGIN "radar_threshold_filter::RadarThresholdFilterNode"
  EXECUTABLE radar_threshold_filter_node
)

## Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
