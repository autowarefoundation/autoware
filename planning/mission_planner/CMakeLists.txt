cmake_minimum_required(VERSION 3.14)
project(mission_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(mission_planner_node SHARED
  lib/mission_planner_base.cpp
  src/mission_planner_lanelet2/mission_planner_lanelet2.cpp
  src/mission_planner_lanelet2/utility_functions.cpp
)

rclcpp_components_register_node(mission_planner_node
  PLUGIN "mission_planner::MissionPlannerLanelet2"
  EXECUTABLE mission_planner
)

ament_auto_add_library(goal_pose_visualizer_node SHARED
  src/goal_pose_visualizer/goal_pose_visualizer.cpp
)

rclcpp_components_register_node(goal_pose_visualizer_node
  PLUGIN "mission_planner::GoalPoseVisualizer"
  EXECUTABLE goal_pose_visualizer
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
