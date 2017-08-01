cmake_minimum_required(VERSION 2.8.3)
project(oxford_ros)

find_package(catkin REQUIRED COMPONENTS
	rospy
)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oxford_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(FILES
	models/*
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

