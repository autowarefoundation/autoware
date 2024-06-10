cmake_minimum_required(VERSION 3.14)
project(autoware_vehicle_info_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(vehicle_info_utils SHARED
  src/vehicle_info.cpp
  src/vehicle_info_utils.cpp
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
