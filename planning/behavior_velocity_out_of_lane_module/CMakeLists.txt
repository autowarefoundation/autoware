cmake_minimum_required(VERSION 3.14)
project(behavior_velocity_out_of_lane_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(behavior_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/decisions.cpp
  src/footprint.cpp
  src/lanelets_selection.cpp
  src/manager.cpp
  src/overlapping_range.cpp
  src/scene_out_of_lane.cpp
)

ament_auto_package()
