cmake_minimum_required(VERSION 2.8.3)
project(map_tools)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
#  runtime_manager
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS runtime_manager
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pcd_filter nodes/pcd_filter/pcd_filter.cpp)
add_executable(pcd_binarizer nodes/pcd_binarizer/pcd_binarizer.cpp)
add_executable(pcd_arealist nodes/pcd_arealist/pcd_arealist.cpp)

target_link_libraries(pcd_filter ${catkin_LIBRARIES})
target_link_libraries(pcd_binarizer ${catkin_LIBRARIES})
target_link_libraries(pcd_arealist ${catkin_LIBRARIES})
