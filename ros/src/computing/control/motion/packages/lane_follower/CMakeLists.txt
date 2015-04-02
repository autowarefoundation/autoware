cmake_minimum_required(VERSION 2.8.3)
project(lane_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  gnss
  message_generation
  runtime_manager
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  lane.msg
  waypoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_follower
  CATKIN_DEPENDS roscpp std_msgs tf runtime_manager
  DEPENDS gnss
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit nodes/pure_pursuit/pure_pursuit.cpp)
target_link_libraries(pure_pursuit gnss ${catkin_LIBRARIES})
add_dependencies(pure_pursuit 
lane_follower_generate_messages_cpp
runtime_manager_generate_messages_cpp)

add_executable(odom_gen nodes/odom_gen/odom_gen.cpp)
target_link_libraries(odom_gen ${catkin_LIBRARIES})
