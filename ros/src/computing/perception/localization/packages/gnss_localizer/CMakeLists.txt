cmake_minimum_required(VERSION 2.8.3)
project(gnss_localizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  gnss
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_follower
  CATKIN_DEPENDS roscpp std_msgs tf 
  DEPENDS gnss
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fix2tfpose nodes/fix2tfpose/fix2tfpose.cpp)
target_link_libraries(fix2tfpose ${catkin_LIBRARIES})

add_executable(nmea2tfpose nodes/nmea2tfpose/nmea2tfpose.cpp)
target_link_libraries(nmea2tfpose gnss ${catkin_LIBRARIES})
