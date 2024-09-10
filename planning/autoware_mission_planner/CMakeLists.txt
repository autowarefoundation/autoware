cmake_minimum_required(VERSION 3.14)
project(autoware_mission_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(goal_pose_visualizer_component SHARED
  src/goal_pose_visualizer/goal_pose_visualizer.cpp
)

rclcpp_components_register_node(goal_pose_visualizer_component
  PLUGIN "autoware::mission_planner::GoalPoseVisualizer"
  EXECUTABLE goal_pose_visualizer
)

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/mission_planner/arrival_checker.cpp
  src/mission_planner/service_utils.cpp
  src/mission_planner/mission_planner.cpp
  src/mission_planner/route_selector.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::mission_planner::MissionPlanner"
  EXECUTABLE mission_planner
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::mission_planner::RouteSelector"
  EXECUTABLE route_selector
)

ament_auto_add_library(${PROJECT_NAME}_lanelet2_plugins SHARED
  src/lanelet2_plugins/default_planner.cpp
  src/lanelet2_plugins/utility_functions.cpp
)
pluginlib_export_plugin_description_file(autoware_mission_planner plugins/plugin_description.xml)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_lanelet2_plugins_default_planner.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_lanelet2_plugins
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
