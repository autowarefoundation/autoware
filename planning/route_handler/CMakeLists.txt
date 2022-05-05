cmake_minimum_required(VERSION 3.5)
project(route_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(route_handler SHARED
  src/route_handler.cpp
)

ament_auto_package()
