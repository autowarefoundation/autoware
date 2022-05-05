cmake_minimum_required(VERSION 3.5)
project(freespace_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(freespace_planner_node SHARED
  src/freespace_planner/freespace_planner_node.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(freespace_planner_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

rclcpp_components_register_node(freespace_planner_node
  PLUGIN "freespace_planner::FreespacePlannerNode"
  EXECUTABLE freespace_planner
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
