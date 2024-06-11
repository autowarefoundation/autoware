cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/planner_manager.cpp
  src/behavior_path_planner_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::behavior_path_planner::BehaviorPathPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_utilities
    test/input.cpp
    test/test_utils.cpp
  )

  target_link_libraries(test_${CMAKE_PROJECT_NAME}_utilities
    ${PROJECT_NAME}_lib
  )

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_node_interface
    test/test_${PROJECT_NAME}_node_interface.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}_node_interface
    ${PROJECT_NAME}_lib
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
