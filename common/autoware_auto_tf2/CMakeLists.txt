cmake_minimum_required(VERSION 3.14)
project(autoware_auto_tf2)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_tf2_autoware_auto_msgs test/test_tf2_autoware_auto_msgs.cpp)
  target_include_directories(test_tf2_autoware_auto_msgs PRIVATE "include")
  ament_target_dependencies(test_tf2_autoware_auto_msgs
    "autoware_auto_common"
    "autoware_auto_perception_msgs"
    "autoware_auto_system_msgs"
    "autoware_auto_geometry_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_geometry_msgs"
    "tf2_ros"
  )
endif()

ament_auto_package()
