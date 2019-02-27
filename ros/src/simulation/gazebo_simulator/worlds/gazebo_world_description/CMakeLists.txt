cmake_minimum_required(VERSION 2.8.3)
project(gazebo_world_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
