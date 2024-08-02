cmake_minimum_required(VERSION 3.14)
project(autoware_autonomous_emergency_braking)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(autoware_autonomous_emergency_braking_helpers SHARED
  include/autoware/autonomous_emergency_braking/utils.hpp
  src/utils.cpp
)

set(AEB_NODE ${PROJECT_NAME}_node)
ament_auto_add_library(${AEB_NODE} SHARED
  include/autoware/autonomous_emergency_braking/node.hpp
  src/node.cpp
)

target_link_libraries(${AEB_NODE} autoware_autonomous_emergency_braking_helpers)
rclcpp_components_register_node(${AEB_NODE}
  PLUGIN "autoware::motion::control::autonomous_emergency_braking::AEB"
  EXECUTABLE ${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_aeb
  test/test.cpp)

target_link_libraries(test_aeb ${AEB_NODE})

endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
