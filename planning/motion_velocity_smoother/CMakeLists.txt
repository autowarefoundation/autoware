cmake_minimum_required(VERSION 3.14)
project(motion_velocity_smoother)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(MOTION_VELOCITY_SMOOTHER_SRC
  src/motion_velocity_smoother_node.cpp
  src/trajectory_utils.cpp
  src/linear_interpolation.cpp
  src/resample.cpp
)

set(SMOOTHER_SRC
  src/smoother/smoother_base.cpp
  src/smoother/l2_pseudo_jerk_smoother.cpp
  src/smoother/linf_pseudo_jerk_smoother.cpp
  src/smoother/jerk_filtered_smoother.cpp
  src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp
  src/smoother/analytical_jerk_constrained_smoother/velocity_planning_utils.cpp
)

ament_auto_add_library(smoother SHARED
  ${SMOOTHER_SRC}
)

ament_auto_add_library(motion_velocity_smoother_node SHARED
  ${MOTION_VELOCITY_SMOOTHER_SRC}
)

target_link_libraries(motion_velocity_smoother_node
  smoother
)

rclcpp_components_register_node(motion_velocity_smoother_node
  PLUGIN "motion_velocity_smoother::MotionVelocitySmootherNode"
  EXECUTABLE motion_velocity_smoother
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)

install(PROGRAMS
  scripts/trajectory_visualizer.py
  scripts/closest_velocity_checker.py
  DESTINATION lib/${PROJECT_NAME}
)
