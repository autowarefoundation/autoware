cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_dynamic_obstacle_stop_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_motion_velocity_planner_node plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

ament_auto_package(INSTALL_TO_SHARE config)
