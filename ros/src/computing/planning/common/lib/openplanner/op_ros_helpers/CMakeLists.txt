cmake_minimum_required(VERSION 2.8.3)
project(op_ros_helpers)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  autoware_msgs  
  map_file
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf
  jsk_recognition_msgs
  op_utility
  op_planner
  op_simu
  waypoint_follower
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op_ros_helpers
  CATKIN_DEPENDS roscpp geometry_msgs autoware_msgs map_file op_utility op_planner op_simu waypoint_follower
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")
if ("${ROS_VERSION}" MATCHES "(kinetic)")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROS_KINETIC")
endif()

include_directories(  
	include
  ${catkin_INCLUDE_DIRS}
)

set(ROS_HELPERS_SRC
		src/PolygonGenerator.cpp
		src/op_RosHelpers.cpp		
)

## Declare a cpp library
add_library(${PROJECT_NAME}
	    ${ROS_HELPERS_SRC}
)

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}		
)
