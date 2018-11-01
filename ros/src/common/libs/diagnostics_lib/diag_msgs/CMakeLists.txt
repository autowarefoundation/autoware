cmake_minimum_required(VERSION 2.8.3)
project(diag_msgs)



find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  message_runtime
)

add_message_files(
  FILES
  diag_error.msg
  diag_node_errors.msg
  diag.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs)

catkin_package(
  CATKIN_DEPENDS
  std_msgs message_runtime
)

foreach(dir msg)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)