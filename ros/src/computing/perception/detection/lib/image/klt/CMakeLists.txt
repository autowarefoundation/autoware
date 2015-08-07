cmake_minimum_required(VERSION 2.8.3)
project(klt)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dpm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(src)

## Declare a cpp library
add_library(klt
   src/klt.cpp
   include/LkTracker.cpp
)

target_link_libraries(klt
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(klt runtime_manager_generate_messages_cpp cv_tracker_generate_messages_cpp)
