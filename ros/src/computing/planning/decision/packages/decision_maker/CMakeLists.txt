cmake_minimum_required(VERSION 2.8.12)
project(decision_maker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-lpthread -pg -Wall)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        std_msgs
        tf
        autoware_msgs
        geometry_msgs
        vector_map_msgs
        vector_map
        jsk_recognition_msgs
        dynamic_reconfigure
        amathutils_lib
        state_machine_lib
        )

catkin_package(
        CATKIN_DEPENDS roscpp
        tf
        std_msgs
        autoware_msgs
        geometry_msgs
        vector_map_msgs
        vector_map
        jsk_recognition_msgs
        vector_map_msgs
        amathutils_lib
        state_machine_lib
)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(decision_maker_node
        nodes/decision_maker/decision_maker_node.cpp
        nodes/decision_maker/decision_maker_node_core.cpp
        nodes/decision_maker/decision_maker_node_decision.cpp
        nodes/decision_maker/decision_maker_node_init.cpp
        nodes/decision_maker/decision_maker_node_callback.cpp
        nodes/decision_maker/decision_maker_node_stateupdate.cpp
        nodes/decision_maker/decision_maker_node_publish.cpp
        nodes/decision_maker/cross_road_area.cpp
        )
target_link_libraries(decision_maker_node ${catkin_LIBRARIES})
add_dependencies(decision_maker_node ${catkin_EXPORTED_TARGETS})

# This function will be integrated into decision_maker soon.
add_executable(planner_selector_node nodes/planner_selector/planner_selector_node.cpp)
target_link_libraries(planner_selector_node ${catkin_LIBRARIES})
add_dependencies(planner_selector_node ${catkin_EXPORTED_TARGETS})

install(TARGETS decision_maker_node planner_selector_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/launch/
        PATTERN ".svn" EXCLUDE
        )
