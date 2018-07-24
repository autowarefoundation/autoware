cmake_minimum_required(VERSION 2.8.3)
project(op_simulator)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  geometry_msgs
  pcl_conversions
  pcl_ros
  op_utility
  op_planner
  op_simu
  autoware_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs op_utility op_planner op_simu autoware_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)
add_executable(op_simulator
        nodes/OpenPlannerSimulator.cpp
        nodes/OpenPlannerSimulator_core.cpp
        nodes/PolygonGenerator.cpp)
target_link_libraries(op_simulator
        ${catkin_LIBRARIES})
add_dependencies(op_simulator
        ${catkin_EXPORTED_TARGETS})

