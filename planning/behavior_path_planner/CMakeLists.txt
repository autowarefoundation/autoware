cmake_minimum_required(VERSION 3.14)
project(behavior_path_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

ament_auto_add_library(behavior_path_planner_node SHARED
  src/planner_manager.cpp
  src/steering_factor_interface.cpp
  src/behavior_path_planner_node.cpp
  src/scene_module/scene_module_visitor.cpp
  src/scene_module/avoidance/avoidance_module.cpp
  src/scene_module/avoidance/manager.cpp
  src/scene_module/dynamic_avoidance/dynamic_avoidance_module.cpp
  src/scene_module/dynamic_avoidance/manager.cpp
  src/scene_module/start_planner/start_planner_module.cpp
  src/scene_module/start_planner/manager.cpp
  src/scene_module/goal_planner/goal_planner_module.cpp
  src/scene_module/goal_planner/manager.cpp
  src/scene_module/side_shift/side_shift_module.cpp
  src/scene_module/side_shift/manager.cpp
  src/scene_module/lane_change/interface.cpp
  src/scene_module/lane_change/normal.cpp
  src/scene_module/lane_change/external_request.cpp
  src/scene_module/lane_change/avoidance_by_lane_change.cpp
  src/scene_module/lane_change/manager.cpp
  src/utils/start_goal_planner_common/utils.cpp
  src/utils/avoidance/shift_line_generator.cpp
  src/utils/avoidance/utils.cpp
  src/utils/lane_change/utils.cpp
  src/utils/side_shift/util.cpp
  src/utils/goal_planner/util.cpp
  src/utils/goal_planner/shift_pull_over.cpp
  src/utils/goal_planner/geometric_pull_over.cpp
  src/utils/goal_planner/freespace_pull_over.cpp
  src/utils/goal_planner/goal_searcher.cpp
  src/utils/goal_planner/default_fixed_goal_planner.cpp
  src/utils/start_planner/util.cpp
  src/utils/start_planner/shift_pull_out.cpp
  src/utils/start_planner/geometric_pull_out.cpp
  src/utils/start_planner/freespace_pull_out.cpp
  src/utils/geometric_parallel_parking/geometric_parallel_parking.cpp
  src/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.cpp
  src/marker_utils/avoidance/debug.cpp
  src/marker_utils/lane_change/debug.cpp
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
    test/test_utils.cpp
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

  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_lane_change_module
    test/test_lane_change_utils.cpp
  )

  target_link_libraries(test_${CMAKE_PROJECT_NAME}_lane_change_module
    behavior_path_planner_node
  )

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_node_interface
    test/test_${PROJECT_NAME}_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_node_interface
    behavior_path_planner_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
