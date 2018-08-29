cmake_minimum_required(VERSION 2.8.13)
project(vision_beyond_track)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        )

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        )

include_directories(
        include
        lib
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_library(
        beyond_track_lib STATIC
        lib/hungarian.cpp
        lib/clipper.cpp
)

add_executable(vision_beyond_track
        src/vision_beyond_track_node.cpp
        src/vision_beyond_track.cpp
)

target_link_libraries(vision_beyond_track
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        beyond_track_lib
)

add_dependencies(vision_beyond_track
        ${catkin_EXPORTED_TARGETS}
)
