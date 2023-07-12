cmake_minimum_required(VERSION 3.14)
project(traffic_light_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_add_library(traffic_light_utils SHARED
  src/traffic_light_utils.cpp
)

ament_auto_package()
