cmake_minimum_required(VERSION 2.8.3)
project(waypoint_maker)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        autoware_msgs
        geometry_msgs
        gnss
        lane_planner
        message_filters
        roscpp
        std_msgs
        tf
        vector_map
        visualization_msgs
        waypoint_follower
)

catkin_package(
    CATKIN_DEPENDS
        autoware_msgs
        geometry_msgs
        lane_planner
        message_filters
        roscpp
        std_msgs
        tf
        vector_map
        visualization_msgs
        waypoint_follower
    DEPENDS gnss
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_loader nodes/waypoint_loader/waypoint_loader_core nodes/waypoint_loader/waypoint_loader_node.cpp)
target_link_libraries(waypoint_loader ${catkin_LIBRARIES})
add_dependencies(waypoint_loader ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_saver nodes/waypoint_saver/waypoint_saver.cpp)
target_link_libraries(waypoint_saver ${catkin_LIBRARIES})
add_dependencies(waypoint_saver ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_clicker nodes/waypoint_clicker/waypoint_clicker.cpp)
target_link_libraries(waypoint_clicker ${catkin_LIBRARIES})
add_dependencies(waypoint_clicker ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_marker_publisher nodes/waypoint_marker_publisher/waypoint_marker_publisher.cpp)
target_link_libraries(waypoint_marker_publisher ${catkin_LIBRARIES})
add_dependencies(waypoint_marker_publisher ${catkin_EXPORTED_TARGETS})

install(TARGETS waypoint_loader waypoint_saver waypoint_clicker waypoint_marker_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
