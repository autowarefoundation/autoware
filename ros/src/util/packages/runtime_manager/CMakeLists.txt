cmake_minimum_required(VERSION 2.8.3)
project(runtime_manager)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  rosgraph
  rosnode
  rospy
  std_msgs
  tablet_socket_msgs
)

catkin_package(
    LIBRARIES runtime_manager
    CATKIN_DEPENDS autoware_msgs geometry_msgs rosgraph rosnode rospy std_msgs tablet_socket_msgs
)

catkin_install_python(PROGRAMS scripts/ftrace.py scripts/runtime_manager_dialog.py scripts/test_srv.py scripts/rtmgr.py scripts/test_pub.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE)
