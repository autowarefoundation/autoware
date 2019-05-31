cmake_minimum_required(VERSION 2.8.12)
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
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        geometry_msgs
        jsk_recognition_msgs
        dynamic_reconfigure
        amathutils_lib
        state_machine_lib
        autoware_msgs
        autoware_config_msgs
        vector_map_msgs
        rostest
        rosunit
        jsk_rviz_plugins
        )

catkin_package(
        CATKIN_DEPENDS roscpp
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
        jsk_rviz_plugins
)
include_directories(
        include/decision_maker
        ${catkin_INCLUDE_DIRS}
        ${vector_map_INCLUDE_DIRS}
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
)

add_executable(decision_maker_node
        nodes/decision_maker/decision_maker_node.cpp
        nodes/decision_maker/decision_maker_node_core.cpp
        nodes/decision_maker/decision_maker_node_decision.cpp
        nodes/decision_maker/decision_maker_node_init.cpp
        nodes/decision_maker/decision_maker_node_callback.cpp
        nodes/decision_maker/decision_maker_node_state_vehicle.cpp
        nodes/decision_maker/decision_maker_node_state_mission.cpp
        nodes/decision_maker/decision_maker_node_state_behavior.cpp
        nodes/decision_maker/decision_maker_node_state_motion.cpp
        nodes/decision_maker/decision_maker_node_publish.cpp
        nodes/decision_maker/cross_road_area.cpp
        )
target_link_libraries(decision_maker_node
        ${catkin_LIBRARIES}
        ${vector_map_LIBRARIES}
        )
add_dependencies(decision_maker_node autoware_msgs_generate_msgs_cpp ${catkin_EXPORTED_TARGETS})

install(TARGETS decision_maker_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
        )

install(FILES
        autoware_state_behavior.yaml
        autoware_state_motion.yaml
        autoware_state_mission.yaml
        autoware_state_vehicle.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-decision_maker
    test/test_decision_maker.test
    test/src/test_main.cpp
    test/src/test_node_decision.cpp
    test/src/test_node_state_drive.cpp
    nodes/decision_maker/decision_maker_node_core.cpp
    nodes/decision_maker/decision_maker_node_decision.cpp
    nodes/decision_maker/decision_maker_node_init.cpp
    nodes/decision_maker/decision_maker_node_callback.cpp
    nodes/decision_maker/decision_maker_node_state_vehicle.cpp
    nodes/decision_maker/decision_maker_node_state_behavior.cpp
    nodes/decision_maker/decision_maker_node_state_motion.cpp
    nodes/decision_maker/decision_maker_node_state_mission.cpp
    nodes/decision_maker/decision_maker_node_publish.cpp
    nodes/decision_maker/cross_road_area.cpp)
  add_dependencies(test-decision_maker ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test-decision_maker ${catkin_LIBRARIES} ${vector_map_LIBRARIES})
endif()
