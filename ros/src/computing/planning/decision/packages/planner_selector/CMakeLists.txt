cmake_minimum_required(VERSION 2.8.3)
project(planner_selector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_msgs
  euc
)

catkin_package(
 INCLUDE_DIRS include
  LIBRARIES planner_selector
  CATKIN_DEPENDS autoware_msgs roscpp std_msgs
  DEPENDS system_lib
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node nodes/planner_selector/planner_selector_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
