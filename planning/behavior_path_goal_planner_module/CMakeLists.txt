cmake_minimum_required(VERSION 3.14)
project(behavior_path_goal_planner_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/default_fixed_goal_planner.cpp
  src/freespace_pull_over.cpp
  src/geometric_pull_over.cpp
  src/goal_searcher.cpp
  src/shift_pull_over.cpp
  src/util.cpp
  src/goal_planner_module.cpp
  src/manager.cpp
)

ament_auto_package(INSTALL_TO_SHARE config)
