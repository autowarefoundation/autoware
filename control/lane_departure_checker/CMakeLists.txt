cmake_minimum_required(VERSION 3.14)
project(lane_departure_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(lane_departure_checker SHARED
  src/lane_departure_checker_node/lane_departure_checker.cpp
  src/lane_departure_checker_node/lane_departure_checker_node.cpp
)

rclcpp_components_register_node(lane_departure_checker
  PLUGIN "lane_departure_checker::LaneDepartureCheckerNode"
  EXECUTABLE lane_departure_checker_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
