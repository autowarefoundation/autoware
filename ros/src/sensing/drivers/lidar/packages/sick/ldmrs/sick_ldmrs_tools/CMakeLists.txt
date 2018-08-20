cmake_minimum_required(VERSION 2.8.3)
project(sick_ldmrs_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  roslaunch
  sick_ldmrs_msgs
  tf
  visualization_msgs
)

find_package(PCL 1.7 REQUIRED REQUIRED)
add_definitions(${PCL_DEFINITIONS})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sick_ldmrs_tools
  CATKIN_DEPENDS roscpp visualization_msgs sick_ldmrs_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(sick_ldmrs_tools
#   src/${PROJECT_NAME}/sick_ldmrs_tools.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(sick_ldmrs_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(sick_ldmrs_object_marker src/sick_ldmrs_object_marker.cpp)
add_executable(sick_ldmrs_filter_layer src/sick_ldmrs_filter_layer.cpp)
add_executable(sick_ldmrs_filter_first src/sick_ldmrs_filter_first.cpp)
add_executable(sick_ldmrs_filter_last src/sick_ldmrs_filter_last.cpp)
add_executable(sick_ldmrs_all_layer_assembler src/sick_ldmrs_all_layer_assembler.cpp)
add_executable(sick_ldmrs_make_organized src/sick_ldmrs_make_organized.cpp)
add_executable(sick_ldmrs_remove_background src/sick_ldmrs_remove_background.cpp)
add_executable(sick_ldmrs_print_resolution src/sick_ldmrs_print_resolution.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(sick_ldmrs_object_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_filter_layer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_filter_first ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_filter_last ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_all_layer_assembler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_make_organized ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_remove_background ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_print_resolution ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(sick_ldmrs_object_marker
  ${catkin_LIBRARIES}
)

target_link_libraries(sick_ldmrs_filter_layer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_filter_first
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_filter_last
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_all_layer_assembler
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_make_organized
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_remove_background
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_print_resolution
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS sick_ldmrs_object_marker sick_ldmrs_filter_layer sick_ldmrs_filter_first sick_ldmrs_filter_last sick_ldmrs_all_layer_assembler sick_ldmrs_make_organized sick_ldmrs_remove_background sick_ldmrs_print_resolution
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sick_ldmrs_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

roslaunch_add_file_check(launch)
