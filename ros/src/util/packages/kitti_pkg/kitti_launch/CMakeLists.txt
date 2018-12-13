cmake_minimum_required(VERSION 3.5)
project(kitti_launch)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS kitti_box_publisher 
)
