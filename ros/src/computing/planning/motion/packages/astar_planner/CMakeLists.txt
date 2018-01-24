cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

# set cpp-11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        autoware_msgs
        geometry_msgs
        nav_msgs
        pcl_conversions
        pcl_ros
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
        nav_msgs
        pcl_conversions
        roscpp
        std_msgs
        tf
        vector_map
        visualization_msgs
        waypoint_follower
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_set nodes/velocity_set/velocity_set.cpp nodes/velocity_set/velocity_set_path.cpp nodes/velocity_set/velocity_set_info.cpp nodes/velocity_set/libvelocity_set.cpp)
target_link_libraries(velocity_set ${catkin_LIBRARIES})
add_dependencies(velocity_set ${catkin_EXPORTED_TARGETS})

add_executable(obstacle_avoid nodes/obstacle_avoid/obstacle_avoid.cpp nodes/obstacle_avoid/astar_search.cpp nodes/obstacle_avoid/search_info_ros.cpp nodes/obstacle_avoid/astar_util.cpp)
target_link_libraries(obstacle_avoid ${catkin_LIBRARIES})
add_dependencies(obstacle_avoid ${catkin_EXPORTED_TARGETS})

add_executable(obstacle_sim nodes/obstacle_avoid/obstacle_sim/obstacle_sim.cpp nodes/obstacle_avoid/obstacle_sim/obstacle_sim_node.cpp)
target_link_libraries(obstacle_sim ${catkin_LIBRARIES})
add_dependencies(obstacle_sim ${catkin_EXPORTED_TARGETS})

install(TARGETS velocity_set obstacle_avoid obstacle_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


