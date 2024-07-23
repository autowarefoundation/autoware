cmake_minimum_required(VERSION 3.5)
project(autoware_radar_threshold_filter)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

## Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/radar_threshold_filter_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::radar_threshold_filter::RadarThresholdFilterNode"
  EXECUTABLE radar_threshold_filter_node
)

## Tests
if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE test_files test/**/*.cpp)
  ament_add_ros_isolated_gtest(${PROJECT_NAME}_test ${test_files})

  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )
endif()

## Package
ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
