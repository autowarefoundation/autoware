cmake_minimum_required(VERSION 2.8.3)
project(vector_map_converter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  amathutils_lib
  autoware_map
  autoware_map_msgs
  geodesy
  geometry_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  roscpp
  vector_map
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vector_map_converter
#  CATKIN_DEPENDS autoware_map geodesy geometry_msgs lanelet2_core lanelet2_io lanelet2_projection lanelet2_routing lanelet2_traffic_rules roscpp vector_map
#  DEPENDS system_lib
)

if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
  set(CMAKE_CXX_STANDARD 17)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(autowaremap2vectormap src/autoware2vectormap.cpp src/autoware2vectormap_core.cpp)
add_executable(lanelet2autowaremap src/lanelet2autowaremap.cpp src/lanelet2autowaremap_core.cpp src/autoware2vectormap_core.cpp)
add_executable(lanelet2vectormap src/lanelet2vectormap.cpp src/lanelet2vectormap_core.cpp src/lanelet2autowaremap_core.cpp src/autoware2vectormap_core.cpp)

add_dependencies(autowaremap2vectormap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lanelet2autowaremap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lanelet2vectormap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(autowaremap2vectormap ${catkin_LIBRARIES})
target_link_libraries(lanelet2autowaremap ${catkin_LIBRARIES})
target_link_libraries(lanelet2vectormap ${catkin_LIBRARIES})

## Install executables and/or libraries
install(TARGETS
        autowaremap2vectormap
        lanelet2autowaremap
        lanelet2vectormap
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(DIRECTORY launch/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#        PATTERN ".svn" EXCLUDE)
