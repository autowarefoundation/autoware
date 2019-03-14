cmake_minimum_required(VERSION 2.8.3)
project(autoware_system_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  rosgraph_msgs
)

add_message_files(
  FILES
  DiagnosticStatus.msg
  DiagnosticStatusArray.msg
  SystemStatus.msg
  NodeStatus.msg
  HardwareStatus.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  rosgraph_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  rosgraph_msgs
)

foreach(dir msg)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)