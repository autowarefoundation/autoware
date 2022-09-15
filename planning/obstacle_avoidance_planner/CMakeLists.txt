cmake_minimum_required(VERSION 3.14)
project(obstacle_avoidance_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_library(obstacle_avoidance_planner SHARED
  src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/costmap_generator.cpp
  src/eb_path_optimizer.cpp
  src/mpt_optimizer.cpp
  src/utils/utils.cpp
  src/utils/debug_utils.cpp
  src/utils/cv_utils.cpp
  src/node.cpp
)

target_include_directories(obstacle_avoidance_planner
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(obstacle_avoidance_planner
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(obstacle_avoidance_planner
  PLUGIN "ObstacleAvoidancePlanner"
  EXECUTABLE obstacle_avoidance_planner_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    scripts
)
