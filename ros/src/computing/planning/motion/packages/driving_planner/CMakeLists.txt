cmake_minimum_required(VERSION 2.8.3)
project(driving_planner)

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
  map_file
  vehicle_socket
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
  LIBRARIES libtraj_gen 
  CATKIN_DEPENDS roscpp std_msgs tf runtime_manager waypoint_follower vehicle_socket map_file #dbw_mkz_msgs
  DEPENDS gnss
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(libtraj_gen lib/libtraj_gen.cpp)
target_link_libraries(libtraj_gen ${catkin_LIBRARIES} armadillo)

add_library(libvelocity_set lib/libvelocity_set.cpp)
add_dependencies(libvelocity_set map_file_generate_messages_cpp)

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

add_executable(velocity_set nodes/velocity_set/velocity_set.cpp)
target_link_libraries(velocity_set libwaypoint_follower libvelocity_set ${catkin_LIBRARIES})
add_dependencies(velocity_set 
waypoint_follower_generate_messages_cpp
runtime_manager_generate_messages_cpp)

add_executable(path_select nodes/path_select/path_select.cpp)
target_link_libraries(path_select libwaypoint_follower  ${catkin_LIBRARIES})
add_dependencies(path_select 
waypoint_follower_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS driving_planner driving_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_driving_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
