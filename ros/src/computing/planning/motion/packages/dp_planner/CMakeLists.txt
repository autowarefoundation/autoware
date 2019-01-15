cmake_minimum_required(VERSION 2.8.3)
project(dp_planner)

find_package(autoware_build_flags REQUIRED)

find_package(vector_map_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        autoware_can_msgs
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
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp
        geometry_msgs
        autoware_can_msgs
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
        #  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
        include include/plannerh
        ${catkin_INCLUDE_DIRS}
        ${vector_map_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(libdp_planner lib/libdp_planner.cpp)

link_directories(lib)

add_executable(dp_planner nodes/dp_planner.cpp nodes/dp_planner_core.cpp nodes/ROSHelpers.cpp nodes/PolygonGenerator.cpp)
target_link_libraries(dp_planner
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        )
add_dependencies(dp_planner
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS dp_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

