cmake_minimum_required(VERSION 2.8.3)
project(gnss)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include/gnss
  LIBRARIES gnss
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include/gnss
)

add_library(gnss
  src/geo_pos_conv.cpp
)

#############
## Install ##
#############

install(
  DIRECTORY include/gnss
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)

## Install executables and/or libraries
install(TARGETS gnss
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
