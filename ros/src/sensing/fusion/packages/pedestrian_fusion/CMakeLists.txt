cmake_minimum_required(VERSION 2.8.3)
project(pedestrian_fusion)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES pedestrian_fusion
   CATKIN_DEPENDS car_fusion # XXX for message
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Declare a cpp executable
add_executable(pedestrian_fusion
  nodes/pedestrian_fusion/pedestrian_fusion.cpp
)

target_link_libraries(pedestrian_fusion
  ${catkin_LIBRARIES} ${OpenCV_LIBS} fusion
)
