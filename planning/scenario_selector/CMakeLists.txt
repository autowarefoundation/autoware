cmake_minimum_required(VERSION 3.5)
project(scenario_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(scenario_selector_node SHARED
  src/scenario_selector_node/scenario_selector_node.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(scenario_selector_node PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

rclcpp_components_register_node(scenario_selector_node
  PLUGIN "ScenarioSelectorNode"
  EXECUTABLE scenario_selector
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
