cmake_minimum_required(VERSION 2.8.3)
project(op_local_planner)

find_package(autoware_build_flags REQUIRED)

find_package(vector_map_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  autoware_can_msgs
  map_file
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf
  jsk_recognition_msgs
  op_utility
  op_planner
  op_simu  
  op_ros_helpers
  waypoint_follower
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  autoware_can_msgs
  map_file
  op_utility
  op_planner
  op_simu
  op_ros_helpers
  waypoint_follower
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${vector_map_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(op_common_params nodes/op_common_params/op_common_params.cpp )
target_link_libraries(op_common_params ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(op_trajectory_generator nodes/op_trajectory_generator/op_trajectory_generator.cpp nodes/op_trajectory_generator/op_trajectory_generator_core.cpp)
target_link_libraries(op_trajectory_generator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(op_trajectory_evaluator nodes/op_trajectory_evaluator/op_trajectory_evaluator.cpp nodes/op_trajectory_evaluator/op_trajectory_evaluator_core.cpp)
target_link_libraries(op_trajectory_evaluator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(op_behavior_selector nodes/op_behavior_selector/op_behavior_selector.cpp nodes/op_behavior_selector/op_behavior_selector_core.cpp)
target_link_libraries(op_behavior_selector ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(op_motion_predictor nodes/op_motion_predictor/op_motion_predictor.cpp nodes/op_motion_predictor/op_motion_predictor_core.cpp)
target_link_libraries(op_motion_predictor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(op_common_params op_trajectory_generator op_trajectory_evaluator op_behavior_selector op_motion_predictor ${catkin_EXPORTED_TARGETS})


install(TARGETS
        op_common_params
        op_trajectory_generator
        op_trajectory_evaluator
        op_behavior_selector
        op_motion_predictor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
