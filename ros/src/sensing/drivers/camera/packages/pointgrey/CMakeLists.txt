cmake_minimum_required(VERSION 2.8.3)
project(pointgrey)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES debug
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

if(EXISTS "/usr/include/flycapture")
include_directories(
	/usr/include/flycapture
	${OpenCV_INCLUDE_DIRS}
)
add_executable(grasshopper3 nodes/grasshopper3/grasshopper3.cpp)

target_link_libraries(grasshopper3
   ${catkin_LIBRARIES} ${OpenCV_LIBS} flycapture
)
else()
message("'flycapture2' is not installed. 'grasshopper3' will not be built.")
endif()


if(EXISTS "/usr/include/ladybug")
include_directories(
	/usr/include/ladybug
	nodes/ladybug
	${OpenCV_INCLUDE_DIRS}
)

add_executable(lady_bug
	nodes/ladybug/ladybug.cpp
)

target_link_libraries(lady_bug
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
   flycapture
   ladybug
   xerces-c
)
else()
message("'lady_bug' is not installed. 'ladybug' will not be built.")
endif()
