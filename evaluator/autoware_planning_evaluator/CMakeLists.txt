cmake_minimum_required(VERSION 3.14)
project(autoware_planning_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)

ament_auto_add_library(planning_evaluator_node SHARED
  DIRECTORY src
)

rclcpp_components_register_node(planning_evaluator_node
  PLUGIN "planning_diagnostics::PlanningEvaluatorNode"
  EXECUTABLE planning_evaluator
)

rclcpp_components_register_node(planning_evaluator_node
  PLUGIN "planning_diagnostics::MotionEvaluatorNode"
  EXECUTABLE motion_evaluator
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_planning_evaluator
    test/test_planning_evaluator_node.cpp
  )
  target_link_libraries(test_planning_evaluator
    planning_evaluator_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
