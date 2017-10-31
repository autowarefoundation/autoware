cmake_minimum_required(VERSION 2.8.3)
project(memsic_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crossbow_imu
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(vg440_node nodes/vg440/vg440_node.cpp)

target_link_libraries(vg440_node ${catkin_LIBRARIES})
