cmake_minimum_required(VERSION 2.8.3)
project(emergency_planner)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  autoware_system_msgs
  roscpp
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES emergency_planner
  CATKIN_DEPENDS actionlib autoware_system_msgs roscpp
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(emergency_planner lib/libemergency_planner.cpp)
target_link_libraries(emergency_planner ${catkin_LIBRARIES})
add_dependencies(emergency_planner ${catkin_EXPORTED_TARGETS})

add_executable(emergency_stop_planner src/emergency_stop_planner.cpp)
target_link_libraries(emergency_stop_planner
  emergency_planner
  ${catkin_LIBRARIES}
)
add_dependencies(emergency_stop_planner ${catkin_EXPORTED_TARGETS})

install(TARGETS emergency_planner emergency_planner emergency_stop_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(DIRECTORY launch/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#         PATTERN ".svn" EXCLUDE)

# catkin_add_gtest(${PROJECT_NAME}-test test/test_emergency_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
