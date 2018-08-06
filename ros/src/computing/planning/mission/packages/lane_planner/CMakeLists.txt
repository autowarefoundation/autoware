cmake_minimum_required(VERSION 2.8.12)
project(lane_planner)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        std_msgs
        tablet_socket
        waypoint_follower
        vector_map
        gnss
        autoware_msgs
        )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES lane_planner
        CATKIN_DEPENDS roscpp
        std_msgs
        tablet_socket
        waypoint_follower
        vector_map
        gnss
        autoware_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(lane_planner lib/lane_planner/lane_planner_vmap.cpp)
target_link_libraries(lane_planner ${catkin_LIBRARIES})
add_dependencies(lane_planner
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(lane_navi nodes/lane_navi/lane_navi.cpp)
target_link_libraries(lane_navi
        lane_planner
        ${catkin_LIBRARIES}
        )
add_dependencies(lane_navi
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(lane_rule nodes/lane_rule/lane_rule.cpp)
target_link_libraries(lane_rule
        lane_planner
        ${catkin_LIBRARIES}
        )
add_dependencies(lane_rule
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(lane_stop nodes/lane_stop/lane_stop.cpp)
target_link_libraries(lane_stop lane_planner ${catkin_LIBRARIES})
add_dependencies(lane_stop
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(lane_select
        nodes/lane_select/lane_select_node.cpp
        nodes/lane_select/lane_select_core.cpp
        nodes/lane_select/hermite_curve.cpp
        )
target_link_libraries(lane_select ${catkin_LIBRARIES})
add_dependencies(lane_select
        ${catkin_EXPORTED_TARGETS}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )

install(TARGETS
        lane_select
        lane_stop
        lane_rule
        lane_navi
        lane_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
