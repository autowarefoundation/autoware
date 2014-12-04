cmake_minimum_required(VERSION 2.8.3)
project(rosinterface)

find_package(catkin REQUIRED)

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

if(EXISTS /usr/local/qtbase5)
# Self install version
set(Qt5INCLUDE "/usr/local/qtbase5/include")
set(Qt5BIN "/usr/local/qtbase5/bin")
link_directories("/usr/local/qtbase5/lib")
else()
# Ubuntu Qt5 package
set(Qt5INCLUDE "/usr/include/qt5")
set(Qt5BIN "/usr/lib/x86_64-linux-gnu/qt5/bin")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/rosinterface
  ${Qt5INCLUDE}
  ${Qt5INCLUDE}/QtCore
  ${Qt5INCLUDE}/QtGui
  ${Qt5INCLUDE}/QtWidgets
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
  Qt5Core
  Qt5Gui
  Qt5Widgets
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
