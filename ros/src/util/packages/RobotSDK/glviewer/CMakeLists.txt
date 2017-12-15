cmake_minimum_required(VERSION 2.8.3)
project(glviewer)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

catkin_package(
  INCLUDE_DIRS include/glviewer
  LIBRARIES glviewer
)

###########
## Build ##
###########

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if("${ARCHITECTURE}" STREQUAL "x86_64" OR "${ARCHITECTURE}" STREQUAL "i686" OR "${ARCHITECTURE}" STREQUAL "aarch64")
    include_directories(
      include/glviewer
      ${catkin_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}
    )

    add_library(glviewer
      GLViewer/glviewer.cpp
      include/glviewer/glviewer.h
    )

    target_link_libraries(glviewer
      ${catkin_LIBRARIES}
      Qt5::Core
      Qt5::Widgets
      Qt5::OpenGL
    )
    #############
    ## Install ##
    #############

    install(
      DIRECTORY include/glviewer
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
)
endif()
