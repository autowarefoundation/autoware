cmake_minimum_required(VERSION 2.8.3)
project(velodyne_pointcloud)

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    dynamic_reconfigure
)


find_package(velodyne_msgs REQUIRED)

find_package(velodyne_driver REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    dynamic_reconfigure
    pcl_conversions)
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

link_directories(${YAML_CPP_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(
  cfg/CloudNode.cfg cfg/TransformNode.cfg
)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(
    include
    ${velodyne_msgs_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

catkin_package(
    CATKIN_DEPENDS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    dynamic_reconfigure
    velodyne_driver
    INCLUDE_DIRS include
    LIBRARIES velodyne_rawdata)
    
#add_executable(dynamic_reconfigure_node src/dynamic_reconfigure_node.cpp)
#target_link_libraries(dynamic_reconfigure_node
#   ${catkin_LIBRARIES}
# )

add_subdirectory(src/lib)
add_subdirectory(src/conversions)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
install(PROGRAMS scripts/gen_calibration.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
