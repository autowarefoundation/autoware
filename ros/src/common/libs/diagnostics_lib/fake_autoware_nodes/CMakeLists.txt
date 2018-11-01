cmake_minimum_required(VERSION 2.8.3)
project(fake_autoware_nodes)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  diag_lib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs diag_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(fake_publisher_node_src src/fake_publisher_node.cpp src/fake_publisher.cpp)
add_executable(fake_publisher_node ${fake_publisher_node_src})
target_link_libraries(fake_publisher_node ${catkin_LIBRARIES})
add_dependencies(fake_publisher_node diag_msgs_generate_messages_cpp)

set(fake_subscriber_node_src src/fake_subscriber_node.cpp src/fake_subscriber.cpp)
add_executable(fake_subscriber_node ${fake_subscriber_node_src})
target_link_libraries(fake_subscriber_node ${catkin_LIBRARIES})
add_dependencies(fake_publisher_node diag_msgs_generate_messages_cpp)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
