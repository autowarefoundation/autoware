cmake_minimum_required(VERSION 2.8.3)
project(emergency_handler)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  autoware_system_msgs
  autoware_health_checker
  roscpp
  std_msgs
  actionlib
)

catkin_package(
  LIBRARIES
  emergency_handler
  system_status_filter
  emergency_plan_client
  CATKIN_DEPENDS
  autoware_system_msgs
  autoware_health_checker
  roscpp
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(system_status_filter lib/libsystem_status_filter.cpp lib/libvital_monitor.cpp)
target_link_libraries(system_status_filter
  ${catkin_LIBRARIES}
)
add_dependencies(system_status_filter ${catkin_EXPORTED_TARGETS})

add_library(emergency_plan_client lib/libemergency_plan_client.cpp)
target_link_libraries(emergency_plan_client
  ${catkin_LIBRARIES}
)
add_dependencies(emergency_plan_client ${catkin_EXPORTED_TARGETS})

add_library(emergency_handler lib/libemergency_handler.cpp)
target_link_libraries(emergency_handler
  system_status_filter
  emergency_plan_client
  ${catkin_LIBRARIES}
)
add_dependencies(emergency_handler ${catkin_EXPORTED_TARGETS})

add_executable(simple_emergency_handler
  src/simple_emergency_handler.cpp
  src/simple_system_status_filter.cpp)
target_link_libraries(simple_emergency_handler
  emergency_handler
  emergency_plan_client
  system_status_filter
  ${catkin_LIBRARIES}
)
add_dependencies(simple_emergency_handler ${catkin_EXPORTED_TARGETS})

install(TARGETS
        emergency_handler
        system_status_filter
        emergency_plan_client
        simple_emergency_handler
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
