cmake_minimum_required(VERSION 3.5)
project(decision_maker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-lpthread -pg -Wall)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

find_package(autoware_build_flags REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(autoware_config_msgs REQUIRED)
find_package(vector_map_msgs REQUIRED)
find_package(vector_map REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jsk_recognition_msgs REQUIRED)
find_package(dynamic_reconfigure REQUIRED)
find_package(amathutils_lib REQUIRED)
find_package(state_machine_lib REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(
        include
        ${vector_map_INCLUDE_DIRS}
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${jsk_recognition_msgs_INCLUDE_DIRS}
        ${dynamic_reconfigure_INCLUDE_DIRS}
        ${amathutils_lib_INCLUDE_DIRS}
        ${state_machine_lib_INCLUDE_DIRS}
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
ament_target_dependencies(decision_maker_node
        roscpp
        tf
        std_msgs
        autoware_config_msgs
        autoware_msgs
        geometry_msgs
        vector_map_msgs
        vector_map
        jsk_recognition_msgs
        amathutils_lib
        state_machine_lib
        )

# This function will be integrated into decision_maker soon.
add_executable(planner_selector_node nodes/planner_selector/planner_selector_node.cpp)
ament_target_dependencies(planner_selector_node
        roscpp
        tf
        std_msgs
        autoware_config_msgs
        autoware_msgs
        geometry_msgs
        vector_map_msgs
        vector_map
        jsk_recognition_msgs
        amathutils_lib
        state_machine_lib
        )

install(TARGETS decision_maker_node planner_selector_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY launch
        DESTINATION include/${PROJECT_NAME}/launch/
        PATTERN ".svn" EXCLUDE
        )

ament_export_dependencies(ament_cmake)
ament_export_dependencies(roscpp)
ament_export_dependencies(tf)
ament_export_dependencies(std_msgs)
ament_export_dependencies(autoware_config_msgs)
ament_export_dependencies(autoware_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(vector_map_msgs)
ament_export_dependencies(vector_map)
ament_export_dependencies(jsk_recognition_msgs)
ament_export_dependencies(vector_map_msgs)
ament_export_dependencies(amathutils_lib)
ament_export_dependencies(state_machine_lib)

ament_package()