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
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES debug
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(/usr/include/flycapture)

add_executable(grasshopper3 nodes/grasshopper3/grasshopper3.cpp)

target_link_libraries(grasshopper3
   ${catkin_LIBRARIES} flycapture
)
