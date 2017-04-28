cmake_minimum_required(VERSION 2.8.3)
project(cv_tracker_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  image_obj.msg
  image_obj_ranged.msg
  image_obj_tracked.msg
  image_rect.msg
  image_rect_ranged.msg
  obj_label.msg
  obj_pose.msg
  ImageObjects.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs
)
