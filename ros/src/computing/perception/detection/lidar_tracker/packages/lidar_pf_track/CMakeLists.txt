cmake_minimum_required(VERSION 2.8.3)
project(lidar_pf_track)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  autoware_msgs
  )


set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  autoware_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

#ParticleFilter Track
add_executable(lidar_pf_track
  nodes/lidar_pf_track/lidar_pf_track.cpp
  )
target_link_libraries(lidar_pf_track
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
add_dependencies(lidar_pf_track
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS
        lidar_pf_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
