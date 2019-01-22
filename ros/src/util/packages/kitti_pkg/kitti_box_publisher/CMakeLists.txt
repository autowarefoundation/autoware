cmake_minimum_required(VERSION 2.8.3)
project(kitti_box_publisher)


find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  jsk_recognition_msgs
  rospy
  sensor_msgs
  std_msgs
  autoware_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs jsk_recognition_msgs rospy sensor_msgs std_msgs autoware_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/kitti_box_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})