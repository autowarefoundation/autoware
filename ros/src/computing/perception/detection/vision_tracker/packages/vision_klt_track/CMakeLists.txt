cmake_minimum_required(VERSION 2.8.12)
project(vision_klt_track)

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
)
###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIRS}
        lib
        lib/lktracker
        include
)


## vision_klt_tracker ##
add_library(lktracker
        lib/lktracker/LkTracker.cpp
        )
target_link_libraries(lktracker
        ${OpenCV_LIBRARIES}
        )
add_dependencies(lktracker
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(vision_klt_track
        nodes/vision_klt_track/vision_klt_track.cpp
        )

target_link_libraries(vision_klt_track
        lktracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_dependencies(vision_klt_track
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS vision_klt_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
