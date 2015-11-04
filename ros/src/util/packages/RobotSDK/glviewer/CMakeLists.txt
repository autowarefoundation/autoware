cmake_minimum_required(VERSION 2.8.3)
project(glviewer)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

pkg_check_modules(Qt5Core REQUIRED Qt5Core)
pkg_check_modules(Qt5Widgets REQUIRED Qt5Widgets)
pkg_check_modules(Qt5OpenGL REQUIRED Qt5OpenGL)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include/glviewer
  LIBRARIES glviewer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
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

if("${ARCHITECTURE}" STREQUAL "x86_64" OR "${ARCHITECTURE}" STREQUAL "i686")
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/glviewer
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
)

add_custom_command(
  OUTPUT glviewer_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/include/glviewer/glviewer_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/glviewer/glviewer.h"
)
add_custom_target(glviewer_moc
  DEPENDS glviewer_moc.cpp
)

add_library(glviewer
  GLViewer/glviewer.cpp
)
add_dependencies(glviewer glviewer_moc)

#############
## Install ##
#############

install(
  DIRECTORY include/glviewer
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
endif()
