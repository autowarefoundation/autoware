cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  runtime_manager
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES libnode_template
  CATKIN_DEPENDS roscpp geometry_msgs runtime_manager
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_machine
  nodes/state_machine/state_machine_node.cpp
  nodes/state_machine/state_machine_core.cpp
  nodes/state_machine/state_machine.cpp)
add_dependencies(state_machine runtime_manager_generate_messages_cpp)
target_link_libraries(state_machine ${catkin_LIBRARIES})

