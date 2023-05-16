cmake_minimum_required(VERSION 3.5)
project(bezier_sampler)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(bezier_sampler SHARED
  src/bezier.cpp
  src/bezier_sampling.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
