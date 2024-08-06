cmake_minimum_required(VERSION 3.14)
project(autoware_twist2accel)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/twist2accel.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::twist2accel::Twist2Accel"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
