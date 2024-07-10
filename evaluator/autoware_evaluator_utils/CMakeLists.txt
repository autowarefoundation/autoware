cmake_minimum_required(VERSION 3.14)
project(autoware_evaluator_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(evaluator_utils SHARED
  src/evaluator_utils.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
)
