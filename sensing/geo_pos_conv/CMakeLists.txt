cmake_minimum_required(VERSION 3.14)
project(geo_pos_conv)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(geo_pos_conv SHARED
  src/geo_pos_conv.cpp
)

ament_auto_package()
