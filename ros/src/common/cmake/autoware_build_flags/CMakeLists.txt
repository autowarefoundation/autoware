cmake_minimum_required(VERSION 3.5)
project(autoware_build_flags)

find_package(ament_cmake REQUIRED)

ament_package(CONFIG_EXTRAS autoware_build_flags-extras.cmake)
