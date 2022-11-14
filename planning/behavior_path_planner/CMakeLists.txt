cmake_minimum_required(VERSION 3.14)
project(behavior_path_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)

ament_auto_add_library(behavior_path_planner_node SHARED
  src/behavior_path_planner_node.cpp
  src/behavior_tree_manager.cpp
  src/utilities.cpp
  src/path_utilities.cpp
  src/debug_utilities.cpp
  src/steering_factor_interface.cpp
  src/turn_signal_decider.cpp
  src/scene_module/scene_module_bt_node_interface.cpp
  src/scene_module/side_shift/side_shift_module.cpp
  src/scene_module/side_shift/util.cpp
  src/scene_module/avoidance/avoidance_module.cpp
  src/scene_module/avoidance/avoidance_utils.cpp
  src/scene_module/avoidance/debug.cpp
  src/scene_module/lane_following/lane_following_module.cpp
  src/scene_module/lane_change/lane_change_module.cpp
  src/scene_module/lane_change/util.cpp
  src/scene_module/lane_change/debug.cpp
  src/scene_module/pull_over/pull_over_module.cpp
  src/scene_module/pull_over/util.cpp
  src/scene_module/pull_over/shift_pull_over.cpp
  src/scene_module/pull_over/geometric_pull_over.cpp
  src/scene_module/pull_over/goal_searcher.cpp
  src/scene_module/pull_out/pull_out_module.cpp
  src/scene_module/pull_out/util.cpp
  src/scene_module/pull_out/shift_pull_out.cpp
  src/scene_module/pull_out/geometric_pull_out.cpp
  src/scene_module/utils/geometric_parallel_parking.cpp
  src/scene_module/utils/occupancy_grid_based_collision_detector.cpp
  src/scene_module/utils/path_shifter.cpp
  src/scene_module/scene_module_visitor.cpp
)

target_include_directories(behavior_path_planner_node SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(behavior_path_planner_node
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(behavior_path_planner_node
  PLUGIN "behavior_path_planner::BehaviorPathPlannerNode"
  EXECUTABLE behavior_path_planner
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_utilities
    test/input.cpp
    test/test_utilities.cpp
  )
  target_link_libraries(test_${CMAKE_PROJECT_NAME}_utilities
    behavior_path_planner_node
  )

  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_avoidance_module
    test/test_avoidance_utils.cpp
  )

  target_link_libraries(test_${CMAKE_PROJECT_NAME}_avoidance_module
    behavior_path_planner_node
  )

  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_turn_signal
    test/test_turn_signal.cpp
  )

  target_link_libraries(test_${CMAKE_PROJECT_NAME}_turn_signal
    behavior_path_planner_node
  )

endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
