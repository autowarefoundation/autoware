cmake_minimum_required(VERSION 2.8.12)
project(vision_beyond_track)

find_package(OpenCV REQUIRED)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        tf
        autoware_msgs
        )

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        tf
        )

include_directories(
        include
        lib
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
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
        autoware_msgs_generate_messages_cpp
)

install(TARGETS beyond_track_lib vision_beyond_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
