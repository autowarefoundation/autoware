cmake_minimum_required(VERSION 2.8.3)
project(waypoint_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf
  gnss
  message_generation
  runtime_manager
)

################################################
## Find OpenMP in order to parallelize loops  ##
################################################

OPTION (USE_OpenMP "Use OpenMP" ON)
IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()
ENDIF()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  lane.msg
  waypoint.msg
  dtlane.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libwaypoint_follower
  CATKIN_DEPENDS roscpp std_msgs tf runtime_manager
  DEPENDS gnss
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(libwaypoint_follower lib/libwaypoint_follower.cpp)
add_dependencies(libwaypoint_follower
waypoint_follower_generate_messages_cpp
runtime_manager_generate_messages_cpp)

#add_library(libtraj_gen lib/libtraj_gen.cpp)
#target_link_libraries(libtraj_gen ${catkin_LIBRARIES} /usr/lib/libarmadillo.so)

add_executable(pure_pursuit nodes/pure_pursuit/pure_pursuit.cpp)
target_link_libraries(pure_pursuit gnss libwaypoint_follower ${catkin_LIBRARIES})
add_dependencies(pure_pursuit 
waypoint_follower_generate_messages_cpp
runtime_manager_generate_messages_cpp)

#add_executable(trajectory_generator nodes/trajectory_generator/trajectory_generator.cpp)
#target_link_libraries(trajectory_generator gnss libwaypoint_follower libtraj_gen ${catkin_LIBRARIES})
#add_dependencies(trajectory_generator
#waypoint_follower_generate_messages_cpp
#runtime_manager_generate_messages_cpp)

#add_executable(command_converter nodes/command_converter/command_converter.cpp)
#target_link_libraries(command_converter libwaypoint_follower libtraj_gen ${catkin_LIBRARIES})
#add_dependencies(command_converter
#waypoint_follower_generate_messages_cpp)

add_executable(odom_gen nodes/odom_gen/odom_gen.cpp)
target_link_libraries(odom_gen libwaypoint_follower ${catkin_LIBRARIES})

add_executable(collision_avoid nodes/collision_avoid/collision_avoid.cpp)
target_link_libraries(collision_avoid libwaypoint_follower ${catkin_LIBRARIES})
add_dependencies(collision_avoid 
waypoint_follower_generate_messages_cpp)

add_executable(twist_through nodes/collision_avoid/twist_through.cpp)
target_link_libraries(twist_through ${catkin_LIBRARIES})

add_executable(velocity_set nodes/velocity_set/velocity_set.cpp)
target_link_libraries(velocity_set ${catkin_LIBRARIES})
add_dependencies(velocity_set
  waypoint_follower_generate_messages_cpp)

