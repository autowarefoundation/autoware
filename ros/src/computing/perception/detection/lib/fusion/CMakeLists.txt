cmake_minimum_required(VERSION 2.8.3)
project(fusion)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fusion
)

include_directories(
  include
)

add_library(fusion
  fusion.cpp
)

target_link_libraries(fusion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  dpm
)

# Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
