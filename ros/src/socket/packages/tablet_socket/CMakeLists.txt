cmake_minimum_required(VERSION 2.8.3)
project(tablet_socket)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  gnss
  roscpp
  std_msgs
  tablet_socket_msgs
  tf
)

catkin_package(
    CATKIN_DEPENDS autoware_msgs geometry_msgs gnss roscpp std_msgs tablet_socket_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tablet_receiver nodes/tablet_receiver/tablet_receiver.cpp)
target_link_libraries(tablet_receiver ${catkin_LIBRARIES})
add_dependencies(tablet_receiver ${catkin_EXPORTED_TARGETS})

add_executable(tablet_sender nodes/tablet_sender/tablet_sender.cpp)
target_link_libraries(tablet_sender ${catkin_LIBRARIES})
add_dependencies(tablet_sender ${catkin_EXPORTED_TARGETS})

install(TARGETS tablet_receiver tablet_sender
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
