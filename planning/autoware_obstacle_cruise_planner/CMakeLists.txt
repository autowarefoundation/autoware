cmake_minimum_required(VERSION 3.14)
project(autoware_obstacle_cruise_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(autoware_obstacle_cruise_planner_core SHARED
  src/node.cpp
  src/utils.cpp
  src/polygon_utils.cpp
  src/optimization_based_planner/velocity_optimizer.cpp
  src/optimization_based_planner/optimization_based_planner.cpp
  src/pid_based_planner/pid_based_planner.cpp
  src/planner_interface.cpp
)

rclcpp_components_register_node(autoware_obstacle_cruise_planner_core
  PLUGIN "autoware::motion_planning::ObstacleCruisePlannerNode"
  EXECUTABLE autoware_obstacle_cruise_planner
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_obstacle_cruise_planner_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
  autoware_obstacle_cruise_planner_core
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

install(PROGRAMS
  scripts/trajectory_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)
