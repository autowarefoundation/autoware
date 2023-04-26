cmake_minimum_required(VERSION 3.14)
project(obstacle_stop_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

ament_auto_add_library(obstacle_stop_planner SHARED
  src/debug_marker.cpp
  src/node.cpp
  src/planner_utils.cpp
  src/adaptive_cruise_control.cpp
)

target_include_directories(obstacle_stop_planner
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(obstacle_stop_planner
    ${PCL_LIBRARIES}
)

rclcpp_components_register_node(obstacle_stop_planner
  PLUGIN "motion_planning::ObstacleStopPlannerNode"
  EXECUTABLE obstacle_stop_planner_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_${PROJECT_NAME}_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
