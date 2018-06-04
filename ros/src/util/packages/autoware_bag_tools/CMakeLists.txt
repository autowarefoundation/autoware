cmake_minimum_required(VERSION 2.8.3)
project(autoware_bag_tools)

find_package(catkin REQUIRED COMPONENTS rospy rosbag)

catkin_package(
  CATKIN_DEPENDS rosbag
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
    scripts/change_frame_id.py
    scripts/replace_msg_time_with_hdr.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
