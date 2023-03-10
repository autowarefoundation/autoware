cmake_minimum_required(VERSION 3.14)
project(planning_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/metrics_calculator.cpp
  src/${PROJECT_NAME}_node.cpp
  src/motion_evaluator_node.cpp
  src/metrics/deviation_metrics.cpp
  src/metrics/metrics_utils.cpp
  src/metrics/obstacle_metrics.cpp
  src/metrics/stability_metrics.cpp
  src/metrics/trajectory_metrics.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "planning_diagnostics::PlanningEvaluatorNode"
  EXECUTABLE ${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "planning_diagnostics::MotionEvaluatorNode"
  EXECUTABLE motion_evaluator
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_planning_evaluator_node.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    param
    launch
)
