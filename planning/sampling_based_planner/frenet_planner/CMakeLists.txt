cmake_minimum_required(VERSION 3.5)
project(frenet_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(frenet_planner SHARED
  DIRECTORY
  src/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
