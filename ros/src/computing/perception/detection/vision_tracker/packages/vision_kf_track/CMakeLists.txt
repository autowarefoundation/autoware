cmake_minimum_required(VERSION 2.8.12)
project(vision_kf_track)

set(ROS_VERSION $ENV{ROS_DISTRO})

include(FindPkgConfig)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  )

FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  cv_bridge
  roscpp
  sensor_msgs
  autoware_msgs
  )
###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIRS}
        lib
        include
)
