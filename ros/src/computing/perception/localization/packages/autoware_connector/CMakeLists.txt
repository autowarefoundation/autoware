cmake_minimum_required(VERSION 2.8.3)
project(autoware_connector)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        geometry_msgs
        autoware_msgs
        nav_msgs
        std_msgs
        )


catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(can_info_translator
        nodes/can_info_translator/can_info_translator_node.cpp
        nodes/can_info_translator/can_info_translator_core.cpp
        )

target_link_libraries(can_info_translator
        ${catkin_LIBRARIES}
        )
add_dependencies(can_info_translator
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS can_info_translator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
