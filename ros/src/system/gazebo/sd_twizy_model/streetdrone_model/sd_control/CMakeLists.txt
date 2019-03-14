cmake_minimum_required(VERSION 2.8.3)
project(sd_control)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    sd_robot
    std_msgs
    std_srvs
	controller_manager
	joint_state_controller
  	robot_state_publisher
  	ackermann_msgs
  	geometry_msgs
  	nav_msgs
  	roscpp
  	sensor_msgs
  	tf
)

catkin_package(
 CATKIN_DEPENDS ackermann_msgs
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
        )

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        FILES_MATCHING PATTERN "*.yaml"
        )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sd_control_node src/robot_control.cpp)

target_link_libraries(sd_control_node
   ${catkin_LIBRARIES}
)


