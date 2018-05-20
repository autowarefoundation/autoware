cmake_minimum_required(VERSION 2.8.3)
project(map_tools)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  velodyne_pointcloud
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

add_executable(latlon2mgrs nodes/coordinate_transformer/latlon2mgrs.cpp nodes/coordinate_transformer/mgrs_converter.cpp)
add_executable(pcd_latlon2mgrs nodes/coordinate_transformer/pcd_latlon2mgrs.cpp nodes/coordinate_transformer/mgrs_converter.cpp)
add_executable(pcd_filter nodes/pcd_filter/pcd_filter.cpp)
add_executable(pcd_binarizer nodes/pcd_binarizer/pcd_binarizer.cpp)
add_executable(pcd_arealist nodes/pcd_arealist/pcd_arealist.cpp)
add_executable(csv2pcd nodes/pcd_converter/csv2pcd.cpp)
add_executable(pcd2csv nodes/pcd_converter/pcd2csv.cpp)
add_executable(map_extender nodes/map_extender/map_extender.cpp)

target_link_libraries(latlon2mgrs ${catkin_LIBRARIES})
target_link_libraries(pcd_latlon2mgrs ${catkin_LIBRARIES})
target_link_libraries(pcd_filter ${catkin_LIBRARIES})
target_link_libraries(pcd_binarizer ${catkin_LIBRARIES})
target_link_libraries(pcd_arealist ${catkin_LIBRARIES})
target_link_libraries(csv2pcd ${catkin_LIBRARIES})
target_link_libraries(pcd2csv ${catkin_LIBRARIES})
target_link_libraries(map_extender ${catkin_LIBRARIES})
