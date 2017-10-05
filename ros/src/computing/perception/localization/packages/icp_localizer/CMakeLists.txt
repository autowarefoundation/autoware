cmake_minimum_required(VERSION 2.8.3)
project(icp_localizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
#  sensor_msgs
  autoware_msgs
  pcl_conversions
  velodyne_pointcloud
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS std_msgs autoware_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(icp_matching nodes/icp_matching/icp_matching.cpp)

target_link_libraries(icp_matching ${catkin_LIBRARIES})

add_dependencies(icp_matching autoware_msgs_generate_messages_cpp)

