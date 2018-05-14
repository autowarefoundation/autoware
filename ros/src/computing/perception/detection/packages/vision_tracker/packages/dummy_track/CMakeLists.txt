cmake_minimum_required(VERSION 2.8.12)
project(dummy_track)

set(ROS_VERSION $ENV{ROS_DISTRO})

include(FindPkgConfig)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  autoware_msgs
)


catkin_package(
  CATKIN_DEPENDS cv_bridge
  roscpp
  autoware_msgs
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

## dummy_tracker ##
add_executable(dummy_track
  nodes/dummy_track/dummy_track.cpp
  )
target_link_libraries(dummy_track
  ${catkin_LIBRARIES}
  )
install(TARGETS dummy_track
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
