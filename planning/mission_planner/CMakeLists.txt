cmake_minimum_required(VERSION 3.14)
project(mission_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(goal_pose_visualizer_component SHARED
  src/goal_pose_visualizer/goal_pose_visualizer.cpp
)

rclcpp_components_register_node(goal_pose_visualizer_component
  PLUGIN "mission_planner::GoalPoseVisualizer"
  EXECUTABLE goal_pose_visualizer
)

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/mission_planner/mission_planner.cpp
  src/mission_planner/arrival_checker.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "mission_planner::MissionPlanner"
  EXECUTABLE ${PROJECT_NAME}
)

ament_auto_add_library(${PROJECT_NAME}_lanelet2_plugins SHARED
  src/lanelet2_plugins/default_planner.cpp
  src/lanelet2_plugins/utility_functions.cpp
)
pluginlib_export_plugin_description_file(mission_planner plugins/plugin_description.xml)

ament_auto_package(
        INSTALL_TO_SHARE
        config
        launch
)
