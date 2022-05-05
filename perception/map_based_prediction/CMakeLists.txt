cmake_minimum_required(VERSION 3.5)
project(map_based_prediction)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(map_based_prediction_node SHARED
  src/map_based_prediction_node.cpp
  src/path_generator.cpp
  src/debug.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(map_based_prediction_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

rclcpp_components_register_node(map_based_prediction_node
  PLUGIN "map_based_prediction::MapBasedPredictionNode"
  EXECUTABLE map_based_prediction
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
