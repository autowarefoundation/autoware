cmake_minimum_required(VERSION 3.14)
project(freespace_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(freespace_planner_node SHARED
  src/freespace_planner/freespace_planner_node.cpp
)

rclcpp_components_register_node(freespace_planner_node
  PLUGIN "freespace_planner::FreespacePlannerNode"
  EXECUTABLE freespace_planner
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_${PROJECT_NAME}_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
