cmake_minimum_required(VERSION 2.8.3)
project(velodyne_hokuyo)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    velodyne_driver
    velodyne_msgs
    autoware_msgs
    laser_geometry
    message_filters
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS} pcl_conversions)
find_package(Boost COMPONENTS signals)


# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})
             
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(include 
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)
    

add_subdirectory(nodes/velodyne_pointcloud)
#add_subdirectory(nodes/velodyne_pointcloud/src/lib)
#add_subdirectory(nodes/velodyne_pointcloud/src/conversions)

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
#install(FILES nodelets.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY launch/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#install(DIRECTORY params/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
#install(PROGRAMS scripts/gen_calibration.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(velodyne_hokuyo nodes/velodyne_hokuyo/velodyne_hokuyo.cpp)
target_link_libraries(velodyne_hokuyo ${catkin_LIBRARIES})
#add_dependencies(velodyne_hokuyo autoware_msgs_generate_messages_cpp)
