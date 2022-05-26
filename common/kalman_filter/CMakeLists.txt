cmake_minimum_required(VERSION 3.14)
project(kalman_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(kalman_filter SHARED
  src/kalman_filter.cpp
  src/time_delay_kalman_filter.cpp
  include/kalman_filter/kalman_filter.hpp
  include/kalman_filter/time_delay_kalman_filter.hpp
)

ament_auto_package()
