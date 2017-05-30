cmake_minimum_required(VERSION 2.8.3)
project(dp_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  runtime_manager
  cv_tracker_msgs
  waypoint_follower_msgs
  map_file
  pcl_conversions
  pcl_ros
  sensor_msgs
  message_generation
  tf
  jsk_recognition_msgs
  op_utilityh
  op_plannerh
  op_simuh
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs runtime_manager cv_tracker_msgs waypoint_follower_msgs map_file op_utilityh op_plannerh op_simuh
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
target_link_libraries(dp_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(dp_planner 
${catkin_EXPORTED_TARGETS}
runtime_manager_generate_messages_cpp
map_file_generate_messages_cpp
lidar_tracker_generate_messages_cpp 
vector_map_server_generate_messages_cpp
)
