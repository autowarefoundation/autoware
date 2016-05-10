cmake_minimum_required(VERSION 2.8.3)
project(distance_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES down_sampling
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(distance_filter src/distance_filter.cpp)

add_dependencies(distance_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(distance_filter ${catkin_LIBRARIES})
