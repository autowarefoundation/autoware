cmake_minimum_required(VERSION 3.5)
project(detection_by_tracker)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

### Find Packages
find_package(ament_cmake_auto REQUIRED)

### Find PCL Dependencies
find_package(PCL REQUIRED QUIET COMPONENTS common search filters segmentation)

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

### Find dependencies listed in the package.xml
ament_auto_find_build_dependencies()

include_directories(
  include
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
