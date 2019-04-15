cmake_minimum_required(VERSION 2.8.3)
project(autoware_map)

find_package(autoware_build_flags REQUIRED)
find_package(autoware_map_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  visualization_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES autoware_map
 CATKIN_DEPENDS roscpp tf visualization_msgs geometry_msgs
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${autoware_map_msgs_INCLUDE_DIRS}
)

add_library( autoware_map lib/autoware_map/map_handler.cpp lib/autoware_map/util.cpp lib/autoware_map/visualization.cpp )
add_dependencies( autoware_map ${catkin_EXPORTED_TARGETS} )
target_link_libraries( autoware_map ${catkin_LIBRARIES} )

## Install executables and/or libraries
install(TARGETS autoware_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
