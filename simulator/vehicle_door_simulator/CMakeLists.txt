cmake_minimum_required(VERSION 3.14)
project(vehicle_door_simulator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(dummy_doors
  src/dummy_doors.cpp
)

ament_auto_package(INSTALL_TO_SHARE launch)
