cmake_minimum_required(VERSION 3.5)
project(behavior_velocity_planner)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Find the non-obvious packages manually
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES
  tier4_api_msgs
  autoware_auto_perception_msgs
  autoware_auto_planning_msgs
  tier4_planning_msgs
  autoware_utils
  Boost
  Eigen3
  diagnostic_msgs
  geometry_msgs
  lanelet2_extension
  PCL
  pcl_conversions
  rclcpp
  sensor_msgs
  interpolation
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  vehicle_info_util
  visualization_msgs
  nav_msgs
  grid_map_ros
)


# Common
ament_auto_add_library(scene_module_lib SHARED
  src/utilization/path_utilization.cpp
  src/utilization/util.cpp
  src/utilization/interpolate.cpp
)

target_include_directories(scene_module_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_lib ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})


# Scene Module: Stop Line
ament_auto_add_library(scene_module_stop_line SHARED
  src/scene_module/stop_line/manager.cpp
  src/scene_module/stop_line/scene.cpp
  src/scene_module/stop_line/debug.cpp
)

target_include_directories(scene_module_stop_line
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_stop_line ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_stop_line scene_module_lib)


# Scene Module: Crosswalk
ament_auto_add_library(scene_module_crosswalk SHARED
  src/scene_module/crosswalk/manager.cpp
  src/scene_module/crosswalk/scene_crosswalk.cpp
  src/scene_module/crosswalk/scene_walkway.cpp
  src/scene_module/crosswalk/debug.cpp
  src/scene_module/crosswalk/util.cpp
)

target_include_directories(scene_module_crosswalk
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_crosswalk ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_crosswalk scene_module_lib)


# Scene Module: Intersection
ament_auto_add_library(scene_module_intersection SHARED
  src/scene_module/intersection/manager.cpp
  src/scene_module/intersection/scene_intersection.cpp
  src/scene_module/intersection/scene_merge_from_private_road.cpp
  src/scene_module/intersection/debug.cpp
  src/scene_module/intersection/util.cpp
)

target_include_directories(scene_module_intersection
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_intersection ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_intersection scene_module_lib)


# Scene Module: Traffic Light
ament_auto_add_library(scene_module_traffic_light SHARED
  src/scene_module/traffic_light/manager.cpp
  src/scene_module/traffic_light/scene.cpp
  src/scene_module/traffic_light/debug.cpp
)

target_include_directories(scene_module_traffic_light
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_traffic_light ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_traffic_light scene_module_lib)


# Scene Module: Blind Spot
ament_auto_add_library(scene_module_blind_spot SHARED
  src/scene_module/blind_spot/manager.cpp
  src/scene_module/blind_spot/scene.cpp
  src/scene_module/blind_spot/debug.cpp
)

target_include_directories(scene_module_blind_spot
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_blind_spot ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_blind_spot scene_module_lib)


# Scene Module: Detection Area
ament_auto_add_library(scene_module_detection_area SHARED
  src/scene_module/detection_area/manager.cpp
  src/scene_module/detection_area/scene.cpp
  src/scene_module/detection_area/debug.cpp
)

target_include_directories(scene_module_detection_area
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_detection_area ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_detection_area scene_module_lib)

# Scene Module: No Stopping Area
ament_auto_add_library(scene_module_no_stopping_area SHARED
  src/scene_module/no_stopping_area/manager.cpp
  src/scene_module/no_stopping_area/scene_no_stopping_area.cpp
  src/scene_module/no_stopping_area/debug.cpp
)

target_include_directories(scene_module_no_stopping_area
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_no_stopping_area ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_no_stopping_area scene_module_lib)


# Scene Module: Virtual Traffic Light
ament_auto_add_library(scene_module_virtual_traffic_light SHARED
  src/scene_module/virtual_traffic_light/manager.cpp
  src/scene_module/virtual_traffic_light/scene.cpp
  src/scene_module/virtual_traffic_light/debug.cpp
)

target_include_directories(scene_module_virtual_traffic_light
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_virtual_traffic_light ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_virtual_traffic_light scene_module_lib)


# SceneModule OcclusionSpot
ament_auto_add_library(scene_module_occlusion_spot SHARED
  src/scene_module/occlusion_spot/grid_utils.cpp
  src/scene_module/occlusion_spot/geometry.cpp
  src/scene_module/occlusion_spot/manager.cpp
  src/scene_module/occlusion_spot/debug.cpp
  src/scene_module/occlusion_spot/scene_occlusion_spot_in_public_road.cpp
  src/scene_module/occlusion_spot/scene_occlusion_spot_in_private_road.cpp
  src/scene_module/occlusion_spot/occlusion_spot_utils.cpp
  src/scene_module/occlusion_spot/risk_predictive_braking.cpp
)

target_include_directories(scene_module_occlusion_spot
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_occlusion_spot ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_occlusion_spot scene_module_lib)

# Scene Module Manager
ament_auto_add_library(scene_module_manager SHARED
  src/planner_manager.cpp
)

target_include_directories(scene_module_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_manager ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_manager scene_module_lib)


# Node
ament_auto_add_library(behavior_velocity_planner SHARED
  src/node.cpp
)

target_include_directories(behavior_velocity_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_velocity_planner ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(behavior_velocity_planner
  scene_module_lib
  scene_module_stop_line
  scene_module_crosswalk
  scene_module_intersection
  scene_module_traffic_light
  scene_module_blind_spot
  scene_module_occlusion_spot
  scene_module_detection_area
  scene_module_no_stopping_area
  scene_module_virtual_traffic_light
  scene_module_manager
)

ament_export_dependencies(${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

rclcpp_components_register_node(behavior_velocity_planner
  PLUGIN "behavior_velocity_planner::BehaviorVelocityPlannerNode"
  EXECUTABLE behavior_velocity_planner_node
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # utils for test
  ament_auto_add_library(utilization SHARED
    src/utilization/util.cpp
  )
  # Gtest for utilization
  ament_add_gtest(utilization-test
    test/src/test_state_machine.cpp
    test/src/test_arc_lane_util.cpp
    test/src/test_utilization.cpp
  )
  target_link_libraries(utilization-test
    gtest_main
    utilization
  )

  # Gtest for occlusion spot
  ament_add_gtest(occlusion_spot-test
    test/src/test_occlusion_spot_utils.cpp
    test/src/test_risk_predictive_braking.cpp
    test/src/test_geometry.cpp
    test/src/test_grid_utils.cpp
    test/src/test_performances.cpp
  )
  target_link_libraries(occlusion_spot-test
    gtest_main
    scene_module_occlusion_spot
  )

endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
