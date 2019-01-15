cmake_minimum_required(VERSION 2.8.3)
project(op_simulation_package)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  autoware_msgs
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
        CATKIN_DEPENDS roscpp geometry_msgs autoware_msgs map_file op_utility op_planner op_simu op_ros_helpers waypoint_follower
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        )

link_directories(lib)


add_executable(op_car_simulator nodes/op_car_simulator/op_car_simulator.cpp nodes/op_car_simulator/op_car_simulator_core.cpp)
target_link_libraries(op_car_simulator
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(op_perception_simulator
        nodes/op_perception_simulator/op_perception_simulator.cpp
        nodes/op_perception_simulator/op_perception_simulator_core.cpp
        )
target_link_libraries(op_perception_simulator
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(op_signs_simulator
        nodes/op_signs_simulator/op_signs_simulator.cpp
        nodes/op_signs_simulator/op_signs_simulator_core.cpp
        )
target_link_libraries(op_signs_simulator
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_dependencies(op_car_simulator
        op_perception_simulator
        op_signs_simulator
        ${catkin_EXPORTED_TARGETS}
        )
        
install(TARGETS
        op_car_simulator
        op_perception_simulator
        op_signs_simulator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
