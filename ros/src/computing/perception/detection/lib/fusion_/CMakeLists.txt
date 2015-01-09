cmake_minimum_required(VERSION 2.8.3)
project(fusion_)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fusion_
)

include_directories(
  include
)

add_library(fusion_
  fusion.cpp
)

target_link_libraries(fusion_
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  dpm
)

# Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
