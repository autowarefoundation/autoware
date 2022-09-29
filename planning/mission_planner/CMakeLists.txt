cmake_minimum_required(VERSION 3.14)
project(mission_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/mission_planner_interface.cpp
  src/mission_planner_lanelet2.cpp
  src/utility_functions.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "mission_planner::MissionPlannerLanelet2"
  EXECUTABLE ${PROJECT_NAME}
)

ament_auto_add_library(goal_pose_visualizer_component SHARED
  src/goal_pose_visualizer.cpp
)

rclcpp_components_register_node(goal_pose_visualizer_component
  PLUGIN "mission_planner::GoalPoseVisualizer"
  EXECUTABLE goal_pose_visualizer
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
