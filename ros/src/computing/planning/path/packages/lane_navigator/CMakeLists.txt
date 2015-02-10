cmake_minimum_required(VERSION 2.8.3)
project(lane_navigator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gnss
  ui_socket
  nav_msgs
  visualization_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_navigator
  DEPENDS gnss ui_socket
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

add_executable(lane_navigator src/lane_navigator.cpp)
target_link_libraries(lane_navigator gnss ${catkin_LIBRARIES})
add_dependencies(lane_navigator ui_socket_generate_messages_cpp)
