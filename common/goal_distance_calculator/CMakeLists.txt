cmake_minimum_required(VERSION 3.5)
project(goal_distance_calculator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

ament_auto_add_library(goal_distance_calculator SHARED
  src/goal_distance_calculator_node.cpp
  src/goal_distance_calculator.cpp
)

rclcpp_components_register_node(goal_distance_calculator
  PLUGIN "goal_distance_calculator::GoalDistanceCalculatorNode"
  EXECUTABLE goal_distance_calculator_node
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
