cmake_minimum_required(VERSION 3.14)
project(behavior_velocity_run_out_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(behavior_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/dynamic_obstacle.cpp
  src/manager.cpp
  src/scene.cpp
  src/state_machine.cpp
  src/utils.cpp
  src/path_utils.cpp
)

ament_auto_package(INSTALL_TO_SHARE config)
