cmake_minimum_required(VERSION 3.14)
project(global_parameter_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_ros_isolated_gtest(test_global_params_launch ${test_files})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
