cmake_minimum_required(VERSION 3.5)
project(planning_evaluator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_${PROJECT_NAME}
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
