cmake_minimum_required(VERSION 2.8.12)
project(vision_kf_track)

set(ROS_VERSION $ENV{ROS_DISTRO})

include(FindPkgConfig)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  autoware_build_flags
  cv_bridge
  roscpp
  sensor_msgs
  autoware_msgs
  )

FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  cv_bridge
  roscpp
  sensor_msgs
  autoware_msgs
  #kf
  )
###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIRS}
        lib
        include
)


## vision_kf_tracker ##

## kf_tracker ##
#if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
#    ADD_EXECUTABLE(kf_track
#            nodes/kf_track/kf_track.cpp
#            )

#    TARGET_LINK_LIBRARIES(kf_track
#            ${catkin_LIBRARIES}
#            ${OpenCV_LIBRARIES}
#            )
#    add_dependencies(kf_track ${catkin_EXPORTED_TARGETS})
#
#install(TARGETS vision_kf_track
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )
#      }
#endif()
