cmake_minimum_required(VERSION 2.8.3)
project(lidar_kf_contour_track)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  jsk_recognition_msgs
  op_ros_helpers
  cv_bridge
  autoware_msgs
  )

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  autoware_msgs
  tf
  jsk_recognition_msgs
  op_ros_helpers
  cv_bridge
  )

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)

#kl contour track
add_executable(lidar_kf_contour_track
  nodes/lidar_kf_contour_track/lidar_kf_contour_track.cpp
  nodes/lidar_kf_contour_track/lidar_kf_contour_track_core.cpp
  nodes/lidar_kf_contour_track/PolygonGenerator.cpp
  nodes/lidar_kf_contour_track/SimpleTracker.cpp
)
target_link_libraries(lidar_kf_contour_track
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(lidar_kf_contour_track
${catkin_EXPORTED_TARGETS}
)

install(TARGETS
        lidar_kf_contour_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
