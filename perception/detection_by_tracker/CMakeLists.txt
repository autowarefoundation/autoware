cmake_minimum_required(VERSION 3.14)
project(detection_by_tracker)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

### Find PCL Dependencies
find_package(PCL REQUIRED)

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# Generate exe file
set(DETECTION_BY_TRACKER_SRC
  src/detection_by_tracker_core.cpp
  src/utils.cpp
)

ament_auto_add_library(detection_by_tracker_node SHARED
  ${DETECTION_BY_TRACKER_SRC}
)

target_link_libraries(detection_by_tracker_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(detection_by_tracker_node
  PLUGIN "DetectionByTracker"
  EXECUTABLE detection_by_tracker
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
