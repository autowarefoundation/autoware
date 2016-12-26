cmake_minimum_required(VERSION 2.8.3)
project(dp_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  runtime_manager
  cv_tracker
  map_file
  pcl_conversions
  pcl_ros
  sensor_msgs
  message_generation
  tf
  jsk_recognition_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs runtime_manager cv_tracker map_file
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include include/plannerh
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(libdp_planner lib/libdp_planner.cpp)

link_directories(lib)

add_executable(dp_planner nodes/dp_planner.cpp nodes/dp_planner_core.cpp nodes/RosHelpers.cpp nodes/PolygonGenerator.cpp)
target_link_libraries(dp_planner utilityh plannerh simuh ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(dp_planner 
runtime_manager_generate_messages_cpp
cv_tracker_generate_messages_cpp
waypoint_follower_generate_messages_cpp
map_file_generate_messages_cpp
lidar_tracker_generate_messages_cpp 
vector_map_server_generate_messages_cpp
)
