cmake_minimum_required(VERSION 2.8.3)
project(points_to_image)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosinterface
  std_msgs
  sensor_msgs
)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

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
#  INCLUDE_DIRS include
#  LIBRARIES points_to_image
   CATKIN_DEPENDS rosinterface
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/qt5
  /usr/include/qt5/QtCore
  /usr/include/qt5/QtGui
  /usr/include/qt5/QtWidgets
)

## Declare a cpp executable
add_executable(points_to_image nodes/points_to_image/main.cpp)
set_target_properties(points_to_image
  PROPERTIES COMPILE_FLAGS "-fPIC"
)

## Specify libraries to link a library or executable target against
target_link_libraries(points_to_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rosinterface
  Qt5Core
  Qt5Gui
  Qt5Widgets
  QtCore
  QtGui
  QtWebKit
)
