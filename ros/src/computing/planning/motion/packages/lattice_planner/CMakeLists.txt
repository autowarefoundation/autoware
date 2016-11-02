cmake_minimum_required(VERSION 2.8.3)
project(lattice_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf
  gnss
  runtime_manager
  waypoint_follower
  vector_map
  vehicle_socket
  astar_planner
  #dbw_mkz_msgs
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


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libtraj_gen libvelocity_set
  CATKIN_DEPENDS roscpp std_msgs tf runtime_manager waypoint_follower vehicle_socket vector_map #dbw_mkz_msgs
  DEPENDS gnss
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(libtraj_gen lib/libtraj_gen.cpp)
target_link_libraries(libtraj_gen ${catkin_LIBRARIES} armadillo)

add_executable(lattice_trajectory_gen nodes/lattice_trajectory_gen/lattice_trajectory_gen.cpp)
target_link_libraries(lattice_trajectory_gen gnss libwaypoint_follower libtraj_gen ${catkin_LIBRARIES})
add_dependencies(lattice_trajectory_gen
waypoint_follower_generate_messages_cpp
runtime_manager_generate_messages_cpp
vehicle_socket_generate_messages_cpp)

add_executable(lattice_twist_convert nodes/lattice_twist_convert/lattice_twist_convert.cpp)
target_link_libraries(lattice_twist_convert libwaypoint_follower libtraj_gen ${catkin_LIBRARIES})
add_dependencies(lattice_twist_convert 
waypoint_follower_generate_messages_cpp
runtime_manager_generate_messages_cpp)

add_executable(lattice_velocity_set nodes/lattice_velocity_set/lattice_velocity_set.cpp)
target_link_libraries(lattice_velocity_set libwaypoint_follower libvelocity_set ${catkin_LIBRARIES})
add_dependencies(lattice_velocity_set 
waypoint_follower_generate_messages_cpp
runtime_manager_generate_messages_cpp)

add_executable(path_select nodes/path_select/path_select.cpp)
target_link_libraries(path_select libwaypoint_follower  ${catkin_LIBRARIES})
add_dependencies(path_select 
waypoint_follower_generate_messages_cpp)

