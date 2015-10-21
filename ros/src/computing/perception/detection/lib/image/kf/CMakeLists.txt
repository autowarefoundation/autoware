cmake_minimum_required(VERSION 2.8.3)
project(kf)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  runtime_manager
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
  CATKIN_DEPENDS runtime_manager
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(src)

## Declare a cpp library
add_library(kf
   src/kf.cpp
)

target_link_libraries(kf
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(kf runtime_manager_generate_messages_cpp  cv_tracker_generate_messages_cpp)
