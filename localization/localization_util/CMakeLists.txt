cmake_minimum_required(VERSION 3.14)
project(localization_util)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(localization_util SHARED
  src/util_func.cpp
  src/tf2_listener_module.cpp
  src/smart_pose_buffer.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_smart_pose_buffer
    test/test_smart_pose_buffer.cpp
    src/smart_pose_buffer.cpp
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
