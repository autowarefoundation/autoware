cmake_minimum_required(VERSION 3.14)
project(emergency_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/emergency_handler/emergency_handler_core.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "EmergencyHandler"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
