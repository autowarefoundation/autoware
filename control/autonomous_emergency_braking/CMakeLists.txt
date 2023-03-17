cmake_minimum_required(VERSION 3.14)
project(autonomous_emergency_braking)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)

set(AEB_NODE ${PROJECT_NAME}_node)
ament_auto_add_library(${AEB_NODE} SHARED
  src/node.cpp
)

rclcpp_components_register_node(${AEB_NODE}
  PLUGIN "autoware::motion::control::autonomous_emergency_braking::AEB"
  EXECUTABLE ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
