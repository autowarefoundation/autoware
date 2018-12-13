cmake_minimum_required(VERSION 3.5)
project(autoware_build_flags)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS autoware_build_flags-extras.cmake)
