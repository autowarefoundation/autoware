cmake_minimum_required(VERSION 2.8.3)
project(points_preprocessor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  cv_bridge
  velodyne_pointcloud
)

catkin_package(CATKIN_DEPENDS
	sensor_msgs
	velodyne_pointcloud
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
find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

add_executable(ground_filter 
	nodes/ground_filter/ground_filter.cpp
)

target_include_directories(ground_filter PRIVATE
	include_directories(${PCL_INCLUDE_DIRS})
)

target_link_libraries(ground_filter
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
