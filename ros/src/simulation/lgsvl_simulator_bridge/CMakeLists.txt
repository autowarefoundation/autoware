cmake_minimum_required(VERSION 2.8.3)
project(lgsvl_simulator_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  rosbridge_server
  roscpp
  rospy
  roslib
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lgsvl_simulator_bridge
  CATKIN_DEPENDS autoware_msgs geometry_msgs rosbridge_server roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

add_executable(lgsvl_simulator_launcher src/lgsvl_simulator_launcher_node.cpp src/lgsvl_simulator_launcher.cpp)
target_link_libraries(lgsvl_simulator_launcher ${catkin_LIBRARIES})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
