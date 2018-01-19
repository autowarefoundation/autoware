cmake_minimum_required(VERSION 2.8.3)
project(lane_planner)

# c++11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        autoware_msgs
        geometry_msgs
        gnss
        roscpp
        std_msgs
        tablet_socket
        tablet_socket_msgs
        tf
        vector_map
        visualization_msgs
        waypoint_follower
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES vmap
    CATKIN_DEPENDS
        autoware_msgs
        geometry_msgs
        roscpp
        std_msgs
        tablet_socket
        tablet_socket_msgs
        tf
        vector_map
        visualization_msgs
        waypoint_follower
    DEPENDS gnss
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vmap lib/lane_planner/vmap.cpp)
target_link_libraries(vmap ${catkin_LIBRARIES})
add_dependencies(vmap ${catkin_EXPORTED_TARGETS})

add_executable(lane_navi nodes/lane_navi/lane_navi.cpp)
target_link_libraries(lane_navi vmap ${catkin_LIBRARIES})
add_dependencies(lane_navi ${catkin_EXPORTED_TARGETS})

add_executable(lane_rule nodes/lane_rule/lane_rule.cpp)
target_link_libraries(lane_rule vmap ${catkin_LIBRARIES})
add_dependencies(lane_rule ${catkin_EXPORTED_TARGETS})

add_executable(lane_stop nodes/lane_stop/lane_stop.cpp)
target_link_libraries(lane_stop vmap ${catkin_LIBRARIES})
add_dependencies(lane_stop ${catkin_EXPORTED_TARGETS})

add_executable(lane_select nodes/lane_select/lane_select_node.cpp nodes/lane_select/lane_select_core.cpp nodes/lane_select/hermite_curve.cpp)
target_link_libraries(lane_select ${catkin_LIBRARIES})
add_dependencies(lane_select ${catkin_EXPORTED_TARGETS})

install(TARGETS lane_navi lane_rule lane_stop lane_select
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS vmap
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn"
        EXCLUDE 
)
