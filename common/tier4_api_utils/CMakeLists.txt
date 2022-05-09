cmake_minimum_required(VERSION 3.14)
project(tier4_api_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  include_directories(include)
  ament_add_ros_isolated_gtest(${PROJECT_NAME}_test test/test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp tier4_external_api_msgs)
endif()

ament_auto_package()
