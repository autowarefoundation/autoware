cmake_minimum_required(VERSION 2.8.3)
project(rosinterface)

include(FindPkgConfig)
find_package(catkin REQUIRED)

pkg_check_modules(Qt5Core REQUIRED Qt5Core)
pkg_check_modules(Qt5Widgets REQUIRED Qt5Widgets)

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
#  CATKIN_DEPENDS other_catkin_pkg
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
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_custom_command(
  OUTPUT rosinterface_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/ROSInterface/rosinterface_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/rosinterface/rosinterface.h"
)
add_custom_target(rosinterface_moc DEPENDS rosinterface_moc.cpp)

## Declare a cpp library
add_library(rosinterface
  ROSInterface/rosinterface.cpp
)

target_link_libraries(rosinterface
  ${catkin_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(rosinterface
  rosinterface_moc
)

#############
## Install ##
#############

install(
  DIRECTORY include/rosinterface
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
