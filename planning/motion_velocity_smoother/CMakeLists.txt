cmake_minimum_required(VERSION 3.14)
project(motion_velocity_smoother)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

set(MOTION_VELOCITY_SMOOTHER_SRC
  src/motion_velocity_smoother_node.cpp
)

set(SMOOTHER_SRC
  src/smoother/smoother_base.cpp
  src/smoother/l2_pseudo_jerk_smoother.cpp
  src/smoother/linf_pseudo_jerk_smoother.cpp
  src/smoother/jerk_filtered_smoother.cpp
  src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp
  src/smoother/analytical_jerk_constrained_smoother/velocity_planning_utils.cpp
  src/trajectory_utils.cpp
  src/resample.cpp
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

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_smoother_functions
  test/test_smoother_functions.cpp
  )
  target_link_libraries(test_smoother_functions
  smoother
  )
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_motion_velocity_smoother_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    motion_velocity_smoother_node
  )
endif()


ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
