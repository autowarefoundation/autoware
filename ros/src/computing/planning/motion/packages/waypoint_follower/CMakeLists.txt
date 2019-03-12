cmake_minimum_required(VERSION 2.8.3)
project(waypoint_follower)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(autoware_config_msgs REQUIRED)

find_package(tablet_socket_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        geometry_msgs
        pcl_ros
        pcl_conversions
        sensor_msgs
        autoware_msgs
        autoware_config_msgs
        tablet_socket_msgs
        autoware_health_checker
        rostest
        rosunit
        )

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libwaypoint_follower
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        geometry_msgs
        autoware_msgs
        autoware_config_msgs
        pcl_ros
        pcl_conversions
        sensor_msgs
        tablet_socket_msgs
        autoware_health_checker
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        include/twist_gate
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${tablet_socket_msgs_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_library(libwaypoint_follower lib/libwaypoint_follower.cpp)
add_dependencies(libwaypoint_follower
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(libwaypoint_follower ${catkin_LIBRARIES})

add_executable(pure_pursuit
        nodes/pure_pursuit/pure_pursuit_node.cpp
        nodes/pure_pursuit/pure_pursuit.cpp
        nodes/pure_pursuit/pure_pursuit_core.cpp
        nodes/pure_pursuit/pure_pursuit_viz.cpp
        )
target_link_libraries(pure_pursuit libwaypoint_follower ${catkin_LIBRARIES})
add_dependencies(pure_pursuit
        ${catkin_EXPORTED_TARGETS})

add_executable(wf_simulator nodes/wf_simulator/wf_simulator.cpp)
target_link_libraries(wf_simulator libwaypoint_follower ${catkin_LIBRARIES})

add_executable(twist_filter nodes/twist_filter/twist_filter.cpp)
target_link_libraries(twist_filter ${catkin_LIBRARIES})
add_dependencies(twist_filter
        ${catkin_EXPORTED_TARGETS})

add_executable(twist_gate nodes/twist_gate/twist_gate_node.cpp nodes/twist_gate/twist_gate.cpp)
target_link_libraries(twist_gate ${catkin_LIBRARIES})
add_dependencies(twist_gate
        ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(TARGETS libwaypoint_follower pure_pursuit wf_simulator twist_filter twist_gate
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test-twist_gate
      test/test_twist_gate.test
      test/src/test_twist_gate.cpp
      nodes/twist_gate/twist_gate.cpp
    )
    add_dependencies(test-twist_gate
      ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-twist_gate
      ${catkin_LIBRARIES})
endif ()
