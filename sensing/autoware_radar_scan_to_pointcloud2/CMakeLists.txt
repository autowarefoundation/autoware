cmake_minimum_required(VERSION 3.5)
project(autoware_radar_scan_to_pointcloud2)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/radar_scan_to_pointcloud2_node.cpp
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::radar_scan_to_pointcloud2::RadarScanToPointcloud2Node"
  EXECUTABLE radar_scan_to_pointcloud2_node
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
  launch
  config
)
