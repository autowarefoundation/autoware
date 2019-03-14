cmake_minimum_required(VERSION 2.8.3)
project(state_machine)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        autoware_msgs
        )


###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS roscpp geometry_msgs autoware_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
)

add_executable(state_machine
        nodes/state_machine/state_machine_node.cpp
        nodes/state_machine/state_machine_core.cpp
        nodes/state_machine/state_machine.cpp)
add_dependencies(state_machine ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine ${catkin_LIBRARIES})

