cmake_minimum_required(VERSION 2.8.3)
project(ndt_tku)
if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_tku
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
)

add_library(ndt_tku src/algebra.cpp src/newton.cpp)

#############
## Install ##
#############

#install(
#  DIRECTORY include
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hh"
#)
endif()
