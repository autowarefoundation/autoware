cmake_minimum_required(VERSION 3.14)
project(behavior_path_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

set(COMPILE_WITH_OLD_ARCHITECTURE TRUE)

set(common_src
  src/utilities.cpp
  src/path_utilities.cpp
  src/steering_factor_interface.cpp
  src/behavior_path_planner_node.cpp
  src/scene_module/scene_module_visitor.cpp
  src/scene_module/avoidance/avoidance_module.cpp
  src/scene_module/pull_out/pull_out_module.cpp
  src/scene_module/pull_over/pull_over_module.cpp
  src/scene_module/side_shift/side_shift_module.cpp
  src/scene_module/lane_change/lane_change_module.cpp
  src/turn_signal_decider.cpp
  src/util/avoidance/util.cpp
  src/util/lane_change/util.cpp
  src/util/side_shift/util.cpp
  src/util/pull_over/util.cpp
  src/util/pull_over/shift_pull_over.cpp
  src/util/pull_over/geometric_pull_over.cpp
  src/util/pull_over/freespace_pull_over.cpp
  src/util/pull_over/goal_searcher.cpp
  src/util/pull_out/util.cpp
  src/util/pull_out/shift_pull_out.cpp
  src/util/pull_out/geometric_pull_out.cpp
  src/util/path_shifter/path_shifter.cpp
  src/util/drivable_area_expansion/drivable_area_expansion.cpp
  src/util/drivable_area_expansion/map_utils.cpp
  src/util/drivable_area_expansion/footprints.cpp
  src/util/drivable_area_expansion/expansion.cpp
  src/util/geometric_parallel_parking/geometric_parallel_parking.cpp
  src/util/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.cpp
  src/marker_util/debug_utilities.cpp
  src/marker_util/avoidance/debug.cpp
  src/marker_util/lane_change/debug.cpp
)

if(COMPILE_WITH_OLD_ARCHITECTURE)
  ament_auto_add_library(behavior_path_planner_node SHARED
    src/behavior_tree_manager.cpp
    src/scene_module/scene_module_bt_node_interface.cpp
    src/scene_module/lane_following/lane_following_module.cpp
    src/scene_module/lane_change/external_request_lane_change_module.cpp
    src/scene_module/pull_over/pull_over_module.cpp
    ${common_src}
  )

  target_compile_definitions(behavior_path_planner_node PRIVATE USE_OLD_ARCHITECTURE)

  message(WARNING "Build behavior_path_planner with OLD framework...")

else()
  ament_auto_add_library(behavior_path_planner_node SHARED
    src/planner_manager.cpp
    src/scene_module/avoidance/manager.cpp
    src/scene_module/pull_out/manager.cpp
    src/scene_module/pull_over/manager.cpp
    src/scene_module/side_shift/manager.cpp
    src/scene_module/lane_change/manager.cpp
    ${common_src}
  )

  message(WARNING "Build behavior_path_planner with NEW framework...")
endif()

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
    test/test_drivable_area_expansion.cpp
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
