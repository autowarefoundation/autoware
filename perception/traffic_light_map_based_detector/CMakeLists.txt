cmake_minimum_required(VERSION 3.5)
project(traffic_light_map_based_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(traffic_light_map_based_detector SHARED
  src/node.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(traffic_light_map_based_detector PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

rclcpp_components_register_node(traffic_light_map_based_detector
  PLUGIN "traffic_light::MapBasedDetector"
  EXECUTABLE traffic_light_map_based_detector_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
