cmake_minimum_required(VERSION 3.14)
project(traffic_light_arbiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_arbiter.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  TrafficLightArbiter
)

ament_auto_package(INSTALL_TO_SHARE launch config)
