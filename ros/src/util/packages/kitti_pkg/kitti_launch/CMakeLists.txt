cmake_minimum_required(VERSION 2.8.3)
project(kitti_launch)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS kitti_box_publisher 
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/xml
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY calib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/calib
        PATTERN ".svn" EXCLUDE)