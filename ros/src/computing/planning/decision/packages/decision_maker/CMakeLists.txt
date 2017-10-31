cmake_minimum_required(VERSION 2.8.3)
project(decision_maker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -lpthread -pg -Wall -Wextra -Wno-unused-parameter)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  autoware_msgs
  geometry_msgs
  state
  vector_map_msgs
  jsk_recognition_msgs
  euc
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_machine
  CATKIN_DEPENDS roscpp  tf std_msgs geometry_msgs autoware_msgs state  vector_map_msgs jsk_recognition_msgs euc
  DEPENDS system_lib
)
include_directories(
 include
 src
  ${catkin_INCLUDE_DIRS}
)

set(TARGET_SRC  nodes/decision_maker/decision_maker_node.cpp
		nodes/decision_maker/decision_maker_node_core.cpp
		nodes/decision_maker/decision_maker_node_decision.cpp
		nodes/decision_maker/decision_maker_node_init.cpp
		nodes/decision_maker/decision_maker_node_callback.cpp
		nodes/decision_maker/decision_maker_node_publish.cpp
		nodes/decision_maker/cross_road_area.cpp
	)


add_executable(${PROJECT_NAME}_node ${TARGET_SRC})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

# This function will be integrated into decision_maker soon.
set(SUBPROJECT_NAME planner_selector)
add_executable(${SUBPROJECT_NAME}_node nodes/planner_selector/planner_selector_node.cpp)
target_link_libraries(${SUBPROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${SUBPROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
