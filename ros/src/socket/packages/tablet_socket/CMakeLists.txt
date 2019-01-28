cmake_minimum_required(VERSION 2.8.3)
project(tablet_socket)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(tablet_socket_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  gnss
  autoware_can_msgs
  geometry_msgs
  autoware_msgs
  )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(CATKIN_DEPENDS
  tf
  tablet_socket_msgs
  autoware_can_msgs
  autoware_msgs
  gnss
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${tablet_socket_msgs_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
)

add_executable(tablet_receiver nodes/tablet_receiver/tablet_receiver.cpp)
target_link_libraries(tablet_receiver ${catkin_LIBRARIES})
add_dependencies(tablet_receiver ${catkin_EXPORTED_TARGETS})

add_executable(tablet_sender nodes/tablet_sender/tablet_sender.cpp)
target_link_libraries(tablet_sender ${catkin_LIBRARIES})
add_dependencies(tablet_sender 
        ${catkin_EXPORTED_TARGETS}
        autoware_msgs_generate_messages_cpp
        )

install(TARGETS tablet_receiver tablet_sender
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
