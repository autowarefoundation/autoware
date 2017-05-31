cmake_minimum_required(VERSION 2.8.3)
project(fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  scan2image
  points2image
  cv_tracker_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fusion
  CATKIN_DEPENDS roscpp scan2image points2image cv_tracker_msgs
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(fusion
  fusion.cpp
  search_distance.cpp
)

target_link_libraries(fusion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(fusion
  points2image_generate_messages_cpp
  cv_tracker_msgs_generate_messages_cpp
  scan2image_generate_messages_cpp
  )

# Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "fusion_func.h"
)
