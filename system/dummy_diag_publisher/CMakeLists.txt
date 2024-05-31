cmake_minimum_required(VERSION 3.14)
project(dummy_diag_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/dummy_diag_publisher_core.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "DummyDiagPublisher"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
