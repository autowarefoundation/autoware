cmake_minimum_required(VERSION 2.8.3)
project(lidar_kf_contour_track)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  autoware_msgs
  tf
  jsk_recognition_msgs
  op_ros_helpers
  )


set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  autoware_msgs
  tf
  jsk_recognition_msgs
  op_ros_helpers
  )

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

#kl contour track
add_executable(lidar_kf_contour_track
  nodes/lidar_kf_contour_track/lidar_kf_contour_track.cpp
  nodes/lidar_kf_contour_track/lidar_kf_contour_track_core.cpp
)
target_link_libraries(lidar_kf_contour_track
${catkin_LIBRARIES}
)
add_dependencies(lidar_kf_contour_track
${catkin_EXPORTED_TARGETS}
)

install(TARGETS
        lidar_kf_contour_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
