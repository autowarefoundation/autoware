cmake_minimum_required(VERSION 2.8.3)
project(pointgrey)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")
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
include_directories(/usr/include/flycapture)
add_executable(grasshopper3 nodes/grasshopper3/grasshopper3.cpp)

target_link_libraries(grasshopper3
   ${catkin_LIBRARIES} flycapture
)
else()
message("'flycapture2' is not installed. 'grasshopper3' is not built.")
endif()
