cmake_minimum_required(VERSION 3.5)
project(steer_offset_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(steer_offset_estimator_node SHARED
  src/steer_offset_estimator_node.cpp
)

rclcpp_components_register_node(steer_offset_estimator_node
  PLUGIN "steer_offset_estimator::SteerOffsetEstimatorNode"
  EXECUTABLE steer_offset_estimator
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
