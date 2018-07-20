cmake_minimum_required(VERSION 2.8.3)
project(op_local_planner)
set(ROS_VERSION $ENV{ROS_DISTRO})


find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
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
  op_ros_helpers
  waypoint_follower
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs autoware_msgs map_file op_utility op_planner op_simu op_ros_helpers waypoint_follower 
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

link_directories(lib)

add_executable(op_trajectory_generator nodes/op_trajectory_generator/op_trajectory_generator.cpp nodes/op_trajectory_generator/op_trajectory_generator_core.cpp)
target_link_libraries(op_trajectory_generator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(op_trajectory_generator 
${catkin_EXPORTED_TARGETS}
)
