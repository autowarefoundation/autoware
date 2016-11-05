cmake_minimum_required(VERSION 2.8.3)
project(points_preprocessor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

#Space Filter
add_executable(space_filter nodes/space_filter/space_filter.cpp)
target_link_libraries(space_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#Ground Filter
add_executable(ground_filter nodes/ground_filter/ground_filter.cpp)
target_link_libraries(ground_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
