cmake_minimum_required(VERSION 2.8.3)
project(tablet_socket_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation)

add_message_files(
  FILES
    Waypoint.msg
    error_info.msg
    gear_cmd.msg
    mode_cmd.msg
    mode_info.msg
    route_cmd.msg
)

generate_messages(
  DEPENDENCIES std_msgs 
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

