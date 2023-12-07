cmake_minimum_required(VERSION 3.14)
project(behavior_path_start_planner_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/start_planner_module.cpp
  src/manager.cpp
  src/freespace_pull_out.cpp
  src/geometric_pull_out.cpp
  src/shift_pull_out.cpp
  src/util.cpp
)

ament_auto_package(INSTALL_TO_SHARE config)
