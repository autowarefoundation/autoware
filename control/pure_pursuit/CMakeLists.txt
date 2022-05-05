cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# pure_pursuit_core
ament_auto_add_library(pure_pursuit_core SHARED
  src/pure_pursuit_core/planning_utils.cpp
  src/pure_pursuit_core/pure_pursuit.cpp
  src/pure_pursuit_core/interpolate.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(pure_pursuit_core PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

# pure_pursuit
ament_auto_add_library(pure_pursuit_node SHARED
  src/pure_pursuit/pure_pursuit_node.cpp
  src/pure_pursuit/pure_pursuit_viz.cpp
)

target_link_libraries(pure_pursuit_node
  pure_pursuit_core
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(pure_pursuit_node PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

rclcpp_components_register_node(pure_pursuit_node
  PLUGIN "pure_pursuit::PurePursuitNode"
  EXECUTABLE pure_pursuit_node_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
