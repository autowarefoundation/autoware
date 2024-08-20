cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_intersection_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_velocity_planner plugins.xml)

find_package(OpenCV REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/manager.cpp
  src/util.cpp
  src/scene_intersection.cpp
  src/intersection_lanelets.cpp
  src/object_manager.cpp
  src/decision_result.cpp
  src/scene_intersection_prepare_data.cpp
  src/scene_intersection_stuck.cpp
  src/scene_intersection_occlusion.cpp
  src/scene_intersection_collision.cpp
  src/scene_merge_from_private_road.cpp
  src/debug.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    ${TEST_SOURCES}
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

install(PROGRAMS
  scripts/ttc.py
  DESTINATION lib/${PROJECT_NAME}
)
