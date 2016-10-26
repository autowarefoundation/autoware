cmake_minimum_required(VERSION 2.8.3)
project(map_file)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss
  std_msgs
  visualization_msgs
  geometry_msgs
  tf
  waypoint_follower
  vector_map
)
pkg_check_modules(PCL_IO REQUIRED pcl_io-1.7)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_drivers
   CATKIN_DEPENDS waypoint_follower std_msgs vector_map
#  DEPENDS system_lib
   DEPENDS gnss curl
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(get_file
  lib/map_file/get_file.cpp
)
add_executable(points_map_loader nodes/points_map_loader/points_map_loader.cpp)
target_link_libraries(points_map_loader ${catkin_LIBRARIES} ${PCL_IO_LIBRARIES} get_file curl)
add_dependencies(points_map_loader waypoint_follower_generate_messages_cpp)

add_executable(vector_map_loader nodes/vector_map_loader/vector_map_loader.cpp)
target_link_libraries(vector_map_loader ${catkin_LIBRARIES} get_file curl vector_map)
