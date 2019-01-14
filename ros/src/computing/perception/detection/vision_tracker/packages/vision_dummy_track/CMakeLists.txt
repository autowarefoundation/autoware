cmake_minimum_required(VERSION 2.8.12)
project(vision_dummy_track)

set(ROS_VERSION $ENV{ROS_DISTRO})

include(FindPkgConfig)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        autoware_msgs
        )


catkin_package(
        CATKIN_DEPENDS roscpp autoware_msgs
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

## vision_dummy_tracker ##
add_executable(vision_dummy_track
        nodes/vision_dummy_track/vision_dummy_track.cpp
        )

target_link_libraries(vision_dummy_track
        ${catkin_LIBRARIES}
        )
add_dependencies(vision_dummy_track
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS vision_dummy_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
