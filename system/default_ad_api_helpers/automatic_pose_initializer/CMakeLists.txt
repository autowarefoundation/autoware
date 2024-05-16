cmake_minimum_required(VERSION 3.14)
project(automatic_pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/automatic_pose_initializer.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "automatic_pose_initializer::AutomaticPoseInitializer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE launch)
