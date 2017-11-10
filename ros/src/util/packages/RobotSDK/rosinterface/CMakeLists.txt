cmake_minimum_required(VERSION 2.8.3)
project(rosinterface)

include(FindPkgConfig)
find_package(catkin REQUIRED
  roscpp
  tf
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/rosinterface
  LIBRARIES rosinterface
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/rosinterface
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(rosinterface
  ROSInterface/rosinterface.cpp
  include/rosinterface/rosinterface.h
)

target_link_libraries(rosinterface
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

#############
## Install ##
#############

install(
  DIRECTORY include/rosinterface
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
