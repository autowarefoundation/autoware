cmake_minimum_required(VERSION 2.8.3)
project(runtime_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  autoware_msgs
  tablet_socket_msgs
)

catkin_package(
  LIBRARIES runtime_manager
  CATKIN_DEPENDS rospy std_msgs autoware_msgs tablet_socket_msgs
)

target_link_libraries("runtime_manager ${catkin_LIBRARIES})
add_dependencies(runtime_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(runtime_manager scripts/runtime_manager.py)
#add_dependencies(runtime_manager runtime_manager_generate_messages_cpp)

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE)
