cmake_minimum_required(VERSION 2.8.3)
project(model_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES model_publisher
#  CATKIN_DEPENDS roscpp tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(model_publisher nodes/model_publisher.cpp)
target_link_libraries(model_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#Mark executables and/or libraries for installation
install(TARGETS model_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/vehicle_model.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
