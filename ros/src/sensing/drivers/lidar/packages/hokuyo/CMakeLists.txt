cmake_minimum_required(VERSION 2.8.3)
project(hokuyo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    autoware_build_flags
    roscpp
    sensor_msgs
    )
find_package(Boost 1.53 REQUIRED system serialization)

## Declare a catkin package
catkin_package(
    DEPENDS roscpp sensor_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(hokuyo_3d nodes/hokuyo_3d/hokuyo_3d.cpp nodes/hokuyo_3d/vssp.hpp)
target_link_libraries(hokuyo_3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})
