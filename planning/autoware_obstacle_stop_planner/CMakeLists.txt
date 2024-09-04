cmake_minimum_required(VERSION 3.14)
project(autoware_obstacle_stop_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY
  src
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  INTERFACE
    src
)

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::motion_planning::ObstacleStopPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_${PROJECT_NAME}_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
