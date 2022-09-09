cmake_minimum_required(VERSION 3.14)
project(behavior_velocity_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Find the non-obvious packages manually
find_package(Boost REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(OpenCV REQUIRED)

set(scene_modules
  detection_area
  blind_spot
  stop_line
  crosswalk
  traffic_light
  intersection
  no_stopping_area
  virtual_traffic_light
  occlusion_spot
  run_out
)

foreach(scene_module IN LISTS scene_modules)
  file(GLOB_RECURSE scene_module_src "src/scene_module/${scene_module}/*")
  list(APPEND scene_modules_src ${scene_module_src})
endforeach()

ament_auto_add_library(behavior_velocity_planner SHARED
  src/node.cpp
  src/planner_manager.cpp
  src/utilization/path_utilization.cpp
  src/utilization/util.cpp
  src/utilization/debug.cpp
  ${scene_modules_src}
)

target_include_directories(behavior_velocity_planner
  SYSTEM PUBLIC
    ${BOOST_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(behavior_velocity_planner
  Boost
  Eigen3
  PCL
)

target_link_libraries(behavior_velocity_planner ${PCL_LIBRARIES})

rclcpp_components_register_node(behavior_velocity_planner
  PLUGIN "behavior_velocity_planner::BehaviorVelocityPlannerNode"
  EXECUTABLE behavior_velocity_planner_node
)

if(BUILD_TESTING)
  # Gtest for utilization
  ament_add_ros_isolated_gtest(utilization-test
    test/src/test_state_machine.cpp
    test/src/test_arc_lane_util.cpp
    test/src/test_utilization.cpp
  )
  target_link_libraries(utilization-test
    gtest_main
    behavior_velocity_planner
  )

  # Gtest for occlusion spot
  ament_add_ros_isolated_gtest(occlusion_spot-test
    test/src/test_occlusion_spot_utils.cpp
    test/src/test_risk_predictive_braking.cpp
    test/src/test_grid_utils.cpp
  )
  target_link_libraries(occlusion_spot-test
    gtest_main
    behavior_velocity_planner
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
