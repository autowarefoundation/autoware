cmake_minimum_required(VERSION 2.8.3)
project(object_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  tf
  sensor_msgs
  nav_msgs
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  autoware_msgs
  vector_map
)
find_package( OpenMP )
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions tf sensor_msgs nav_msgs autoware_msgs vector_map
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(laserscan2costmap nodes/laserscan2costmap/laserscan2costmap.cpp)
target_link_libraries(laserscan2costmap ${catkin_LIBRARIES})

add_executable(points2costmap nodes/points2costmap/points2costmap.cpp)
target_link_libraries(points2costmap ${catkin_LIBRARIES})

add_executable(potential_field nodes/potential_field/potential_field.cpp)
add_dependencies(potential_field autoware_msgs_generate_messages_cpp)
target_link_libraries(potential_field ${catkin_LIBRARIES})

add_executable(grid_map_filter
  nodes/grid_map_filter/grid_map_filter.cpp
  nodes/grid_map_filter/grid_map_filter_node.cpp
)
target_link_libraries(grid_map_filter ${catkin_LIBRARIES} vector_map)

add_executable(wayarea2grid
  nodes/wayarea2grid/wayarea2grid.cpp
  nodes/wayarea2grid/wayarea2grid_node.cpp
)
target_link_libraries(wayarea2grid ${catkin_LIBRARIES} vector_map)
