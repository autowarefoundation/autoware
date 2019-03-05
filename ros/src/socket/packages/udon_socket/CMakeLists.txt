cmake_minimum_required(VERSION 2.8.3)
project(udon_socket)


find_package(autoware_build_flags REQUIRED)

find_package(tablet_socket_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES udon
        CATKIN_DEPENDS tablet_socket_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${tablet_socket_msgs_INCLUDE_DIRS}
)

add_library(udon lib/udon_socket/udon.cpp)

add_executable(udon_receiver nodes/udon_receiver/udon_receiver.cpp)
target_link_libraries(udon_receiver udon ${catkin_LIBRARIES})

add_executable(udon_sender nodes/udon_sender/udon_sender.cpp)
target_link_libraries(udon_sender udon ${catkin_LIBRARIES})
add_dependencies(udon_sender
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS udon  udon_receiver udon_sender
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})