cmake_minimum_required(VERSION 3.14)
project(obstacle_avoidance_planner)

include(CheckCXXCompilerFlag)

# For Eigen vectorization.
check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if(COMPILER_SUPPORTS_MARCH_NATIVE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
  message(STATUS "Enabling MARCH NATIVE ")
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(obstacle_avoidance_planner SHARED
  # node
  src/node.cpp
  # core algorithms
  src/replan_checker.cpp
  src/eb_path_smoother.cpp
  src/mpt_optimizer.cpp
  src/state_equation_generator.cpp
  # debug marker
  src/debug_marker.cpp
  # vehicle model
  src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  # utils
  src/utils/trajectory_utils.cpp
  src/utils/geometry_utils.cpp
)

target_include_directories(obstacle_avoidance_planner
  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

# register node
rclcpp_components_register_node(obstacle_avoidance_planner
  PLUGIN "obstacle_avoidance_planner::ObstacleAvoidancePlanner"
  EXECUTABLE obstacle_avoidance_planner_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    scripts
    config
)
