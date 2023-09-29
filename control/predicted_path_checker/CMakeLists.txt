cmake_minimum_required(VERSION 3.14)
project(predicted_path_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  include
  SYSTEM
  ${Eigen3_INCLUDE_DIRS}
)

ament_auto_add_library(predicted_path_checker SHARED
  src/predicted_path_checker_node/predicted_path_checker_node.cpp
  src/predicted_path_checker_node/collision_checker.cpp
  src/predicted_path_checker_node/utils.cpp
  src/predicted_path_checker_node/debug_marker.cpp

)

rclcpp_components_register_node(predicted_path_checker
  PLUGIN "autoware::motion::control::predicted_path_checker::PredictedPathCheckerNode"
  EXECUTABLE predicted_path_checker_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
