cmake_minimum_required(VERSION 2.8.3)
project(catvehicle)
set(ROS_VERSION $ENV{ROS_DISTRO})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        controller_manager
        gazebo_ros_control
        geometry_msgs
        position_controllers
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf
        transmission_interface
        velocity_controllers
        velodyne_pointcloud
        )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Need to have these in order to find/include boost stuff
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include(FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
endif ()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)

catkin_python_setup()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES catvehicle
        CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs velodyne_pointcloud tf
)

include_directories(include include/catvehicle)
include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_library(catvehiclegazebo SHARED src/cont.cc ./include/catvehicle/cont.hh)
target_link_libraries(catvehiclegazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(distanceEstimator src/distanceEstimator.cpp)
target_link_libraries(distanceEstimator ${catkin_LIBRARIES})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS catvehiclegazebo
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
