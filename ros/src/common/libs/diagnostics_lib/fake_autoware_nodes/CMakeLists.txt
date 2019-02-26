cmake_minimum_required(VERSION 3.5)
project(fake_autoware_nodes)

add_compile_options(-std=c++11)

find_package(geometry_msgs REQUIRED)
find_package(roscpp REQUIRED)
find_package(rospy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diag_lib REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${diag_lib_INCLUDE_DIRS}
)

set(fake_publisher_node_src src/fake_publisher_node.cpp src/fake_publisher.cpp)
add_executable(fake_publisher_node ${fake_publisher_node_src})
ament_target_dependencies(fake_publisher_node
  diag_msgs_generate_messages_cpp
  roscpp
  diag_lib
)

set(fake_subscriber_node_src src/fake_subscriber_node.cpp src/fake_subscriber.cpp)
add_executable(fake_subscriber_node ${fake_subscriber_node_src})
ament_target_dependencies(fake_subscriber_node
  diag_msgs_generate_messages_cpp
  roscpp
  diag_lib
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(roscpp)
ament_export_dependencies(rospy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(diag_lib)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()