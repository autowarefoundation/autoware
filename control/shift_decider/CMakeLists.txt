cmake_minimum_required(VERSION 3.5)
project(shift_decider)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

ament_auto_add_library(shift_decider_node SHARED
  src/shift_decider.cpp
)

rclcpp_components_register_node(shift_decider_node
  PLUGIN "ShiftDecider"
  EXECUTABLE shift_decider
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
