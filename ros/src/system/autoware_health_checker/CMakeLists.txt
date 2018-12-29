cmake_minimum_required(VERSION 2.8.3)
project(autoware_health_checker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_system_msgs
  roscpp
  diagnostic_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES node_status_publisher
#  CATKIN_DEPENDS autoware_system_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(node_status_publisher
  src/node_status_publisher.cpp
  src/diag_buffer.cpp
  src/rate_checker.cpp
)

# CPP Execution programs
#set(CPP_EXEC_NAMES watchdog_node)
#foreach(cpp_exec_names ${CPP_EXEC_NAMES})
#  install(TARGETS ${cpp_exec_names}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )
#endforeach(cpp_exec_names)

# include header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library
install(TARGETS node_status_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)