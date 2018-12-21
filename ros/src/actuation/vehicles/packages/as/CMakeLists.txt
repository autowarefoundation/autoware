cmake_minimum_required(VERSION 2.8.3)
project(as)

find_package(
        catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        message_filters
        std_msgs
        geometry_msgs
        automotive_platform_msgs
        dbw_mkz_msgs
        autoware_msgs
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        message_filters
        std_msgs
        geometry_msgs
        automotive_platform_msgs
        dbw_mkz_msgs
        autoware_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(as_interface
        nodes/as_interface/as_interface_node.cpp
        nodes/as_interface/as_interface.cpp
        )
target_link_libraries(as_interface ${catkin_LIBRARIES})
add_dependencies(as_interface ${catkin_EXPORTED_TARGETS})

install(TARGETS as_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
