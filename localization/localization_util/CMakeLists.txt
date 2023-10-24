cmake_minimum_required(VERSION 3.14)
project(localization_util)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(localization_util SHARED
  src/util_func.cpp
  src/pose_array_interpolator.cpp
  src/tf2_listener_module.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
)
