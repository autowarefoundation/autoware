cmake_minimum_required(VERSION 2.8.3)
project(ring_filter)

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

add_executable(ring_filter src/ring_filter.cpp)

add_dependencies(ring_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ring_filter
  ${catkin_LIBRARIES}
)
