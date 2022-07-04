cmake_minimum_required(VERSION 3.14)
project(pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

# pure_pursuit_core
ament_auto_add_library(pure_pursuit_core SHARED
  src/pure_pursuit_core/planning_utils.cpp
  src/pure_pursuit_core/pure_pursuit.cpp
  src/pure_pursuit_core/interpolate.cpp
)

# pure_pursuit
ament_auto_add_library(pure_pursuit_lateral_controller SHARED
  src/pure_pursuit/pure_pursuit_lateral_controller.cpp
  src/pure_pursuit/pure_pursuit_viz.cpp
)

target_link_libraries(pure_pursuit_lateral_controller
  pure_pursuit_core
)

rclcpp_components_register_node(pure_pursuit_lateral_controller
  PLUGIN "pure_pursuit::PurePursuitLateralController"
  EXECUTABLE pure_pursuit_lateral_controller_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
