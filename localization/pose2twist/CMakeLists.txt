cmake_minimum_required(VERSION 3.14)
project(pose2twist)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose2twist_core.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Pose2Twist"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
