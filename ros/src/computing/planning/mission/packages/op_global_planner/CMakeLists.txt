cmake_minimum_required(VERSION 2.8.3)
project(op_global_planner)


find_package(autoware_build_flags REQUIRED)

find_package(vector_map_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS

  geometry_msgs
  roscpp
  rospy
  std_msgs
  waypoint_follower
  message_generation
  tf
  jsk_recognition_msgs
  pcl_conversions
  pcl_ros
  op_utility
  op_planner
  op_simu
  op_ros_helpers
  autoware_can_msgs
)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op_global_planner
  CATKIN_DEPENDS geometry_msgs vector_map_msgs roscpp rospy std_msgs waypoint_follower op_utility op_planner op_simu op_ros_helpers autoware_can_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${vector_map_INCLUDE_DIRS}
)



## Declare a C++ executable
 add_executable(op_global_planner nodes/op_global_planner.cpp nodes/op_global_planner_core.cpp)
target_link_libraries(op_global_planner ${catkin_LIBRARIES} )

add_dependencies(op_global_planner ${catkin_EXPORTED_TARGETS})

install(TARGETS
        op_global_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
