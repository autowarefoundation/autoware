cmake_minimum_required(VERSION 2.8.3)
project(kitti_launch)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS kitti_box_publisher 
)
