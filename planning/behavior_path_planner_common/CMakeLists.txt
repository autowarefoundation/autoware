cmake_minimum_required(VERSION 3.14)
project(behavior_path_planner_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

ament_auto_add_library(behavior_path_planner_common SHARED
  src/turn_signal_decider.cpp
  src/utils/utils.cpp
  src/utils/path_utils.cpp
  src/utils/traffic_light_utils.cpp
  src/utils/path_safety_checker/safety_check.cpp
  src/utils/path_safety_checker/objects_filtering.cpp
  src/utils/path_shifter/path_shifter.cpp
  src/utils/drivable_area_expansion/static_drivable_area.cpp
  src/utils/drivable_area_expansion/drivable_area_expansion.cpp
  src/utils/drivable_area_expansion/map_utils.cpp
  src/utils/drivable_area_expansion/footprints.cpp
  src/marker_utils/utils.cpp
)

target_include_directories(behavior_path_planner_common SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(behavior_path_planner_common
  ${OpenCV_LIBRARIES}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_utilities
    test/test_drivable_area_expansion.cpp
  )

  target_link_libraries(test_${CMAKE_PROJECT_NAME}_utilities
    behavior_path_planner_common
  )

  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_safety_check
    test/test_safety_check.cpp
  )

  target_link_libraries(test_${CMAKE_PROJECT_NAME}_safety_check
    behavior_path_planner_common
  )

  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_turn_signal
    test/test_turn_signal.cpp
  )

  target_link_libraries(test_${CMAKE_PROJECT_NAME}_turn_signal
    behavior_path_planner_common
  )
endif()

ament_auto_package()
