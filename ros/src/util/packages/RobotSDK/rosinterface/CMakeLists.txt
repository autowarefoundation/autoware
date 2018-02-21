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

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)


catkin_package(
  INCLUDE_DIRS include/rosinterface
  LIBRARIES rosinterface
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

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

#############
## Install ##
#############

install(
  DIRECTORY include/rosinterface
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
