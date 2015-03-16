cmake_minimum_required(VERSION 2.8.3)
project(glviewer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

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

if("${ARCHITECTURE}" STREQUAL "x86_64")
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/glviewer
  ${Qt5INCLUDE}
  ${Qt5INCLUDE}/QtCore
  ${Qt5INCLUDE}/QtGui
  ${Qt5INCLUDE}/QtOpenGL
  ${Qt5INCLUDE}/QtWidgets
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
