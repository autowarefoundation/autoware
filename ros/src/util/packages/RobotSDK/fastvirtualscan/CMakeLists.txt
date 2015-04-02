cmake_minimum_required(VERSION 2.8.3)
project(fastvirtualscan)

find_package(catkin)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include/fastvirtualscan
  LIBRARIES fastvirtualscan
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result ${CMAKE_CXX_FLAGS}")

if(EXISTS /usr/local/qtbase5)
# Self install version
set(Qt5INCLUDE "/usr/local/qtbase5/include")
set(Qt5BIN "/usr/local/qtbase5/bin")
set(Qt5LIBROOT "/usr/local/qtbase5/lib")
else()
# Ubuntu Qt5 package
set(Qt5INCLUDE "/usr/include/qt5")
set(Qt5BIN "/usr/lib/x86_64-linux-gnu/qt5/bin")
set(Qt5LIBROOT "/usr/lib/x86_64-linux-gnu")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/fastvirtualscan
  ${Qt5INCLUDE}
  ${Qt5INCLUDE}/QtCore
  ${Qt5INCLUDE}/QtGui
  ${Qt5INCLUDE}/QtOpenGL
  ${Qt5INCLUDE}/QtWidgets
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
