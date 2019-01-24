cmake_minimum_required(VERSION 2.8.3)
project(rosbag_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rosbag_storage rosconsole roscpp topic_tools xmlrpcpp rviz)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()


find_package(Boost REQUIRED COMPONENTS date_time regex program_options filesystem thread)
find_package(BZip2 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
link_directories(${catkin_LIBRARY_DIRS})

##include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_definitions(-D_FILE_OFFSET_BITS=64)
add_definitions(-DQT_NO_KEYWORDS)
add_definitions(${QT_DEFINITIONS})

set(CMAKE_CXX_FLAGS "-Wall -g -O0")

set(SOURCE_FILES
  src/core/recorder.cpp
)

set(UI_FILES
  src/autoware_rosbag_plugin.cpp
)

set(FORM_FILES
  src/autoware_rosbag_plugin.ui
)

QT5_WRAP_CPP(QT_MOC_SRC_HPP
  src/autoware_rosbag_plugin.h
)

QT5_WRAP_UI(QT_FORMS_HPP
  ${FORM_FILES}
)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP
  ${RESOURCE_FILES}
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
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosbag_storage rosconsole roscpp rviz topic_tools xmlrpcpp
  DEPENDS Boost
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
  ${BOOST_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a cpp library
add_library( ${PROJECT_NAME}
  ${QT_MOC_SRC_HPP}
  ${QT_FORMS_HPP}
  ${SOURCE_FILES}
  ${UI_FILES}
  ${QT_RESOURCES_CPP}
)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME}
   ${QT_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   yaml-cpp
   rosbag
)

find_package (class_loader)
class_loader_hide_library_symbols( ${PROJECT_NAME} )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )



## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
