cmake_minimum_required(VERSION 3.5)
project(goal_distance_calculator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(goal_distance_calculator SHARED
  src/goal_distance_calculator_node.cpp
  src/goal_distance_calculator.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(goal_distance_calculator PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

rclcpp_components_register_node(goal_distance_calculator
  PLUGIN "goal_distance_calculator::GoalDistanceCalculatorNode"
  EXECUTABLE goal_distance_calculator_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
