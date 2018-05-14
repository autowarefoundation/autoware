cmake_minimum_required(VERSION 2.8.3)
project(lidar_kf_contour_track)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  autoware_msgs
  tf
  jsk_recognition_msgs
  op_ros_helpers
  )


set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  pcl_ros
  roscpp
  autoware_msgs
  tf
  jsk_recognition_msgs
  op_ros_helpers
  )

include_directories(
        ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

#kl contour track
add_executable(lidar_kf_contour_track
lidar_kf_contour_track.cpp
src/lidar_kf_contour_track_core.cpp
)
target_link_libraries(lidar_kf_contour_track
${catkin_LIBRARIES}
${PCL_LIBRARIES}
)
add_dependencies(lidar_kf_contour_track
${catkin_EXPORTED_TARGETS}
)
