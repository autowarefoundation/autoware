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

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/glviewer
  /usr/include/qt5
  /usr/include/qt5/QtCore
  /usr/include/qt5/QtGui
  /usr/include/qt5/QtOpenGL
  /usr/include/qt5/QtWidgets
)

add_custom_command(
  OUTPUT glviewer_moc.cpp
  COMMAND
  "/usr/lib/x86_64-linux-gnu/qt5/bin/moc"
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
