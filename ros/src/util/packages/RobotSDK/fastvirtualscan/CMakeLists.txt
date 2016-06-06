cmake_minimum_required(VERSION 2.8.3)
project(fastvirtualscan)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

pkg_check_modules(Qt5Core REQUIRED Qt5Core)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include/fastvirtualscan
  LIBRARIES fastvirtualscan
  CATKIN_DEPENDS roscpp sensor_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/fastvirtualscan
  ${Qt5Core_INCLUDE_DIRS}
)

add_library(fastvirtualscan
  FastVirtualScan/fastvirtualscan.cpp
)

#############
## Install ##
#############

install(
  DIRECTORY include/fastvirtualscan
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
