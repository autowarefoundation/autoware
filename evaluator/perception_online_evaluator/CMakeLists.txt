cmake_minimum_required(VERSION 3.14)
project(perception_online_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/metrics_calculator.cpp
  src/${PROJECT_NAME}_node.cpp
  src/metrics/deviation_metrics.cpp
  src/metrics/detection_count.cpp
  src/utils/marker_utils.cpp
  src/utils/objects_filtering.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "perception_diagnostics::PerceptionOnlineEvaluatorNode"
  EXECUTABLE ${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "perception_diagnostics::MotionEvaluatorNode"
  EXECUTABLE motion_evaluator
)

target_link_libraries(${PROJECT_NAME}_node glog::glog)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_perception_online_evaluator_node.cpp
    TIMEOUT 300
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
