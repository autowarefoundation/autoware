cmake_minimum_required(VERSION 3.14)
project(vehicle_info_util)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(vehicle_info_util SHARED
  src/vehicle_info_util.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)

install(PROGRAMS
  scripts/min_turning_radius_calculator.py
  DESTINATION lib/${PROJECT_NAME}
)
