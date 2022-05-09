cmake_minimum_required(VERSION 3.14)
project(autoware_point_types)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_autoware_point_types
    test/test_point_types.cpp
  )
  target_include_directories(test_autoware_point_types
    PRIVATE include
  )
  ament_target_dependencies(test_autoware_point_types
    point_cloud_msg_wrapper
  )
endif()

ament_auto_package()
