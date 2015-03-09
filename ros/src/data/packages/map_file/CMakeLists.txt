cmake_minimum_required(VERSION 2.8.3)
project(map_file)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss
  std_msgs
  visualization_msgs
  geometry_msgs
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

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/pcl-1.7
  /usr/include/eigen3)

add_executable(points_map_loader nodes/points_map_loader.cpp)
target_link_libraries(points_map_loader ${catkin_LIBRARIES} pcl_io)

add_executable(vector_map_loader nodes/vector_map_loader.cpp)
target_link_libraries(vector_map_loader ${catkin_LIBRARIES})
