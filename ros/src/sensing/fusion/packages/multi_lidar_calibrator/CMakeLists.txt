cmake_minimum_required(VERSION 2.8.12)
project(multi_lidar_calibrator)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        pcl_ros
        pcl_conversions
        )

catkin_package(CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        geometry_msgs
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# MultiLidar Calibrator
add_library(multi_lidar_calibrator_lib SHARED
        src/multi_lidar_calibrator.cpp
        include/multi_lidar_calibrator.h)

target_include_directories(multi_lidar_calibrator_lib PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

target_link_libraries(multi_lidar_calibrator_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

add_executable(multi_lidar_calibrator
        src/multi_lidar_calibrator_node.cpp
        )

target_include_directories(multi_lidar_calibrator PRIVATE
        include)

target_link_libraries(multi_lidar_calibrator
        multi_lidar_calibrator_lib)
