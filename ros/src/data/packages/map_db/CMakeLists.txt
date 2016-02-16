cmake_minimum_required(VERSION 2.8.3)
project(map_db)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss
  map_file
)
pkg_check_modules(PCL_IO REQUIRED pcl_io-1.7)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -Wno-unused-result ${CMAKE_CXX_FLAGS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_drivers
   CATKIN_DEPENDS map_file
#  DEPENDS system_lib
   DEPENDS gnss
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

add_library(map_db
  lib/map_db/GetFile.cpp
)

add_executable(points_map_query nodes/points_map_query/points_map_query.cpp)
target_link_libraries(points_map_query ${catkin_LIBRARIES} ${PCL_IO_LIBRARIES} map_db)

add_executable(vector_map_query
  nodes/vector_map_query/vector_map_query.cpp
  nodes/vector_map_query/pack_into.cpp)
target_link_libraries(vector_map_query ${catkin_LIBRARIES} map_db)
add_dependencies(vector_map_query map_file_generate_messages_cpp)
