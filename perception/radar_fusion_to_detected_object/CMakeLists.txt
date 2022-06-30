cmake_minimum_required(VERSION 3.5)
project(radar_fusion_to_detected_object)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

## Targets
ament_auto_add_library(radar_object_fusion_to_detected_object_node_component SHARED
  src/radar_object_fusion_to_detected_object_node/radar_object_fusion_to_detected_object_node.cpp
  src/radar_fusion_to_detected_object.cpp
)

rclcpp_components_register_node(radar_object_fusion_to_detected_object_node_component
  PLUGIN "radar_fusion_to_detected_object::RadarObjectFusionToDetectedObjectNode"
  EXECUTABLE radar_object_fusion_to_detected_object_node
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
