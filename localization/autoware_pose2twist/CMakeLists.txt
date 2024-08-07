cmake_minimum_required(VERSION 3.14)
project(autoware_pose2twist)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose2twist_core.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose2twist::Pose2Twist"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_angular_velocity
    test/test_angular_velocity.cpp
  )
  target_include_directories(test_angular_velocity PRIVATE
    src
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
