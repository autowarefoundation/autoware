cmake_minimum_required(VERSION 2.8.3)
project(decision_maker)

# set cpp-11
set(CMAKE_CXX_STANDARD 11)

# This function will be integrated into decision_maker soon.
set(SUBPROJECT_NAME planner_selector)

find_package(catkin REQUIRED COMPONENTS
        amathutils
        autoware_msgs
        geometry_msgs
        jsk_recognition_msgs
        jsk_rviz_plugins
        message_generation
        roscpp
        sensor_msgs
        state
        std_msgs
        tf
        vector_map
        vector_map_msgs
        visualization_msgs
)

add_message_files(
	FILES
	DebugDecisionMaker.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        amathutils
        autoware_msgs
        geometry_msgs
        jsk_recognition_msgs
        jsk_rviz_plugins
        message_runtime
        roscpp
        sensor_msgs
        state
        std_msgs
        tf
        vector_map
        vector_map_msgs
        visualization_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
                nodes/decision_maker/decision_maker_node.cpp
                nodes/decision_maker/decision_maker_node_core.cpp
                nodes/decision_maker/decision_maker_node_decision.cpp
                nodes/decision_maker/decision_maker_node_init.cpp
                nodes/decision_maker/decision_maker_node_callback.cpp
                nodes/decision_maker/decision_maker_node_stateupdate.cpp
                nodes/decision_maker/decision_maker_node_publish.cpp
                nodes/decision_maker/cross_road_area.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(${SUBPROJECT_NAME}_node nodes/planner_selector/planner_selector_node.cpp)
target_link_libraries(${SUBPROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${SUBPROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

install(TARGETS ${PROJECT_NAME}_node ${SUBPROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)



