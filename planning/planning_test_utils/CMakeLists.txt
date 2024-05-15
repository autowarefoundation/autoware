cmake_minimum_required(VERSION 3.14)
project(planning_test_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(INSTALL_TO_SHARE
  config
  test_map
)
