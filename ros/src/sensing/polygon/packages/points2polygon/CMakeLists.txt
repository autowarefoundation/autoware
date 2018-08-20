cmake_minimum_required(VERSION 2.8.3)
project(points2polygon)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        sensor_msgs
        autoware_msgs
        pcl_conversions
        )

find_package(PCL)
find_package(Qt5Core REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        CATKIN_DEPENDS
        roscpp
        sensor_msgs
        autoware_msgs
        pcl_conversions
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(points2polygon nodes/points2polygon/points2polygon.cpp)
target_link_libraries(points2polygon ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(points2polygon ${catkin_EXPORTED_TARGETS})
