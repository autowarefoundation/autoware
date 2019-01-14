cmake_minimum_required(VERSION 2.8.3)
project(lidar_kf_track)


find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  pcl_ros
  roscpp
  std_msgs
  autoware_msgs
  tf
  jsk_recognition_msgs
  jsk_rviz_plugins
  )

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  pcl_ros
  roscpp
  std_msgs
  autoware_msgs
  tf
  jsk_recognition_msgs
  jsk_rviz_plugins
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  )
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#KalmanFilter Track

add_executable(lidar_kf_track
  nodes/lidar_kf_track/lidar_kf_track_core.cpp
  nodes/lidar_kf_track/lidar_kf_track.cpp
  nodes/lidar_kf_track/hungarian_alg.cpp
  nodes/lidar_kf_track/kalman.cpp
  )
target_link_libraries(lidar_kf_track
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
add_dependencies(lidar_kf_track
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS
        lidar_kf_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
