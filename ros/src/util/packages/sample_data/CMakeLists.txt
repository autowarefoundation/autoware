cmake_minimum_required(VERSION 2.8.3)
project(sample_data)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss
  std_msgs
  visualization_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_drivers
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
   DEPENDS gnss
)

###########
## Build ##
###########

add_executable(sample_vector_map nodes/sample_vector_map/sample_vector_map.cpp)
target_link_libraries(sample_vector_map ${catkin_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/pcl-1.7
  /usr/include/eigen3)

add_executable(sample_points_map nodes/sample_points_map/sample_points_map.cpp)
target_link_libraries(sample_points_map ${catkin_LIBRARIES} pcl_io)

add_executable(sample_points_map_update nodes/sample_points_map_update/sample_points_map_update.cpp)
target_link_libraries(sample_points_map_update ${catkin_LIBRARIES} pcl_io)

add_executable(sample_trajectory nodes/sample_trajectory/sample_trajectory.cpp)
target_link_libraries(sample_trajectory ${catkin_LIBRARIES} gnss)

add_executable(sample_mobility nodes/sample_mobility/sample_mobility.cpp)
target_link_libraries(sample_mobility ${catkin_LIBRARIES} gnss)
