cmake_minimum_required(VERSION 2.8.3)
project(points_localizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  runtime_manager
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS runtime_manager
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ndt_pcl src/ndt_pcl.cpp)
add_executable(ndt_slam src/ndt_slam.cpp)

target_link_libraries(ndt_pcl ${catkin_LIBRARIES})
target_link_libraries(ndt_slam ${catkin_LIBRARIES})
add_dependencies(ndt_pcl runtime_manager_generate_messages_cpp)
add_dependencies(ndt_slam runtime_manager_generate_messages_cpp)
