cmake_minimum_required(VERSION 2.8.3)
project(op_ros_helpers)
set(ROS_VERSION $ENV{ROS_DISTRO})


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(vector_map_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        map_file
        pcl_conversions
        pcl_ros
        sensor_msgs
        tf
        jsk_recognition_msgs
        op_utility
        op_planner
        op_simu
        waypoint_follower
        autoware_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES op_ros_helpers
        CATKIN_DEPENDS roscpp
        geometry_msgs
        autoware_msgs
        map_file
        pcl_conversions
        pcl_ros
        sensor_msgs
        tf
        jsk_recognition_msgs
        op_utility
        op_planner
        op_simu
        waypoint_follower
        vector_map_msgs
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")
if ("${ROS_VERSION}" MATCHES "(kinetic)")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROS_KINETIC")
endif ()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${vector_map_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
)

set(ROS_HELPERS_SRC
        src/PolygonGenerator.cpp
        src/op_ROSHelpers.cpp
        )

## Declare a cpp library
add_library(${PROJECT_NAME}
        ${ROS_HELPERS_SRC}
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


install(TARGETS op_ros_helpers
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
