cmake_minimum_required(VERSION 2.8.3)
project(autoware_health_checker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_system_msgs
  roscpp
  diagnostic_msgs
  jsk_rviz_plugins
  rostest
  rosunit
  rosgraph_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES node_status_publisher
  CATKIN_DEPENDS autoware_system_msgs roscpp rosgraph_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_system_msgs_INCLUDE_DIRS}
)

set(NODE_STATUS_PUBLISHER_SRC
  src/node_status_publisher.cpp
  src/diag_buffer.cpp
  src/rate_checker.cpp
)
add_library(node_status_publisher
  ${NODE_STATUS_PUBLISHER_SRC}
)
target_link_libraries(node_status_publisher ${catkin_LIBRARIES})
add_dependencies(node_status_publisher ${catkin_EXPORTED_TARGETS} autoware_system_msgs_generate_messages_cpp)

add_library(system_status_subscriber
  src/system_status_subscriber.cpp
)
target_link_libraries(system_status_subscriber ${catkin_LIBRARIES})
add_dependencies(system_status_subscriber ${catkin_EXPORTED_TARGETS} autoware_system_msgs_generate_messages_cpp)

add_executable(health_aggregator
  src/health_aggregator_node.cpp
  src/health_aggregator.cpp
)
target_link_libraries(health_aggregator ${catkin_LIBRARIES})
add_dependencies(health_aggregator ${catkin_EXPORTED_TARGETS} autoware_system_msgs_generate_messages_cpp)

add_executable(health_analyzer
  src/health_analyzer_node.cpp
  src/health_analyzer.cpp
)
target_link_libraries(health_analyzer ${catkin_LIBRARIES})
add_dependencies(health_analyzer ${catkin_EXPORTED_TARGETS} autoware_system_msgs_generate_messages_cpp)

# CPP Execution programs
set(CPP_EXEC_NAMES health_aggregator health_analyzer)
foreach(cpp_exec_names ${CPP_EXEC_NAMES})
  install(TARGETS ${cpp_exec_names}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endforeach(cpp_exec_names)

# include header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library
install(TARGETS node_status_publisher system_status_subscriber
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-autoware_health_checker
  test/test_autoware_health_checker.test
  test/src/test_autoware_health_checker.cpp
  ${NODE_STATUS_PUBLISHER_SRC})
  target_link_libraries(test-autoware_health_checker
  ${catkin_LIBRARIES})
endif ()