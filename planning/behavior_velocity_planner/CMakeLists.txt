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

# concatenate include_dirs
set(${PROJECT_NAME}_INCLUDE_DIRS
  ${${PROJECT_NAME}_FOUND_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
list(REMOVE_DUPLICATES ${PROJECT_NAME}_INCLUDE_DIRS)

# concatenate link libraries
set(${PROJECT_NAME}_LINK_LIBRARIES ${${PROJECT_NAME}_FOUND_LIBRARIES} ${PCL_LIBRARIES})
list(REMOVE_DUPLICATES ${PROJECT_NAME}_LINK_LIBRARIES)

# utilization library
add_library(behavior_velocity_planner_utilization
  SHARED
  src/utilization/path_utilization.cpp
  src/utilization/util.cpp
  src/utilization/debug.cpp
)
target_include_directories(behavior_velocity_planner_utilization
  SYSTEM PUBLIC
  ${${PROJECT_NAME}_INCLUDE_DIRS}
)
target_link_libraries(behavior_velocity_planner_utilization ${${PROJECT_NAME}_LINK_LIBRARIES})

# scene modules libraries
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
  speed_bump
  out_of_lane
)

set(behavior_velocity_planner_scene_modules behavior_velocity_planner_utilization)
foreach(scene_module IN LISTS scene_modules)
  file(GLOB_RECURSE scene_module_src "src/scene_module/${scene_module}/*")
  add_library(behavior_velocity_planner_${scene_module}
    SHARED ${scene_module_src}
  )
  target_include_directories(behavior_velocity_planner_${scene_module}
    SYSTEM PUBLIC
    ${${PROJECT_NAME}_INCLUDE_DIRS}
  )
  target_link_libraries(behavior_velocity_planner_${scene_module} ${${PROJECT_NAME}_LINK_LIBRARIES} behavior_velocity_planner_utilization)
  list(APPEND behavior_velocity_planner_scene_modules behavior_velocity_planner_${scene_module})
endforeach()

# node library
add_library(behavior_velocity_planner SHARED
  src/node.cpp
  src/planner_manager.cpp
)
target_include_directories(behavior_velocity_planner
  SYSTEM PUBLIC
  ${${PROJECT_NAME}_INCLUDE_DIRS}
)
target_link_libraries(behavior_velocity_planner ${${PROJECT_NAME}_LINK_LIBRARIES} ${behavior_velocity_planner_scene_modules})

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
    ${${PROJECT_NAME}_LINK_LIBRARIES} ${behavior_velocity_planner_scene_modules}
  )

  # Gtest for occlusion spot
  ament_add_ros_isolated_gtest(occlusion_spot-test
    test/src/test_occlusion_spot_utils.cpp
    test/src/test_risk_predictive_braking.cpp
    test/src/test_grid_utils.cpp
  )
  # Gtest for out of lane
  # ament_add_ros_isolated_gtest(out_of_lane-test
  # )
  target_link_libraries(occlusion_spot-test
    gtest_main
    behavior_velocity_planner
    ${${PROJECT_NAME}_LINK_LIBRARIES} ${behavior_velocity_planner_scene_modules}
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
