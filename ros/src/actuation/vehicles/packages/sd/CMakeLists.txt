cmake_minimum_required(VERSION 2.8.3)
project(sd)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        rosconsole
        std_msgs
        autoware_msgs
        can_msgs
        socketcan_interface
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -w")

catkin_package(
  INCLUDE_DIRS
    include/
  LIBRARIES
    socketcan_to_topic
    topic_to_socketcan
  CATKIN_DEPENDS roscpp std_msgs autoware_msgs can_msgs socketcan_interface rosconsole
)


include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
)

add_executable(sd_twizy_interface_node
        src/sd_twizy_interface/sd_twizy_interface_node.cpp)
target_link_libraries(sd_twizy_interface_node
        ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsd_twizy_interface_lib.so)

# socketcan_to_topic
add_library(socketcan_to_topic
        src/socketcan_bridge/socketcan_to_topic.cpp
        )
target_link_libraries(socketcan_to_topic
        ${catkin_LIBRARIES}
        )
add_dependencies(socketcan_to_topic
        ${catkin_EXPORTED_TARGETS}
        )

# topic_to_socketcan
add_library(topic_to_socketcan
        src/socketcan_bridge/topic_to_socketcan.cpp
        )
target_link_libraries(topic_to_socketcan
        ${catkin_LIBRARIES}
        )
add_dependencies(topic_to_socketcan
        ${catkin_EXPORTED_TARGETS}
        )

# topic_to_socketcan_node
add_executable(socketcan_bridge_node
  src/socketcan_bridge/socketcan_bridge_node.cpp

)
target_link_libraries(socketcan_bridge_node
        socketcan_to_topic
        topic_to_socketcan
  ${catkin_LIBRARIES}
)

install(
  TARGETS
       socketcan_to_topic
        topic_to_socketcan
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

