cmake_minimum_required(VERSION 3.14)
project(ad_api_adaptors)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/initial_pose_adaptor.cpp
  src/routing_adaptor.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ad_api_adaptors::InitialPoseAdaptor"
  EXECUTABLE initial_pose_adaptor_node
  EXECUTOR MultiThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ad_api_adaptors::RoutingAdaptor"
  EXECUTABLE routing_adaptor_node
  EXECUTOR SingleThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE config launch)
