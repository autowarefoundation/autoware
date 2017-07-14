cmake_minimum_required(VERSION 2.8.3)
project(velodyne_compression)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
  pcl_ros
  sensor_msgs
  velodyne_msgs
  pcl_conversions
  )
find_package( OpenCV REQUIRED )


set(${compression}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    velodyne_driver
    velodyne_msgs
    dynamic_reconfigure
)
#set(${PROJECT_NAME}_CATKIN_DEPS
#    angles
#    nodelet
#    pcl_ros
#    roscpp
#    roslib
#    sensor_msgs
#    tf
#    velodyne_driver
#    velodyne_msgs
#    dynamic_reconfigure
#)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  DIRECTORY msg
  FILES
  CompressedPacket.msg
)
generate_messages(DEPENDENCIES std_msgs)

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
#add_message_files(
  #DIRECTORY
 # msg
#  FILES
#  Particles.msg
#  PState.msg
#)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs#   visualization_msgs
# )
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   visualization_msgs
# )
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
#  LIBRARIES particle_filter
  CATKIN_DEPENDS message_runtime std_msgs
  #roscpp rospy std_msgs tf visualization_msgs
  DEPENDS system_lib message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare a cpp library
# add_library(particle_filter
#   src/${PROJECT_NAME}/particle_filter.cpp
# )

## Declare a cpp executable
# add_executable(particle_filter_node src/particle_filter_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(particle_filter_node particle_filter_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(particle_filter_node
#   ${catkin_LIBRARIES}
# )
#add_executable(velodyne_listener src/listen_velodyne.cpp)
#target_link_libraries(velodyne_listener ${catkin_LIBRARIES})

#add_executable(velodyne_time src/velodyne_time.cpp)
#target_link_libraries(velodyne_time ${catkin_LIBRARIES})

#add_executable(mapper src/mapping.cpp)
#target_link_libraries(mapper ${catkin_LIBRARIES})
#target_link_libraries(mapper ${PCL_LIBRARIES})

#add_executable(depth src/depth.cpp)
#target_link_libraries(depth ${catkin_LIBRARIES})
#target_link_libraries(depth ${PCL_LIBRARIES})

#add_executable(deskew src/deskew.cpp)
#target_link_libraries(deskew ${catkin_LIBRARIES})

#add_executable(objectdetector src/objectdetector.cpp)
#target_link_libraries(objectdetector ${catkin_LIBRARIES})

#add_executable(packet_sample src/packet_sample.cpp)
#target_link_libraries(packet_sample ${catkin_LIBRARIES})
#target_link_libraries(packet_sample ${PCL_LIBRARIES})
#target_link_libraries(packet_sample ${OpenCV_LIBRARIES})

add_executable(compress2png src/compress2png.cpp)
target_link_libraries(compress2png ${catkin_LIBRARIES})
target_link_libraries(compress2png ${PCL_LIBRARIES})
target_link_libraries(compress2png ${OpenCV_LIBRARIES})

add_executable(recon2packet src/recon2packet.cpp)
target_link_libraries(recon2packet ${catkin_LIBRARIES})
target_link_libraries(recon2packet ${PCL_LIBRARIES})
target_link_libraries(recon2packet ${OpenCV_LIBRARIES})
target_link_libraries(recon2packet ${libpcap_LIBRARIES})


#add_message_files(
#  DIRECTORY msg
#  FILES
#  CompressedPacket.msg
#)

#add_message_files(FILES CompressedPacket.msg)
#add_executable(packet_rewrite src/packet_rewrite.cpp)
#target_link_libraries(packet_rewrite ${catkin_LIBRARIES})
#target_link_libraries(packet_rewrite ${PCL_LIBRARIES})

#add_executable(cali src/cali.cpp)
#target_link_libraries(cali ${catkin_LIBRARIES})
#target_link_libraries(cali ${PCL_LIBRARIES})

#target_link_libraries(objectdetector ${PCL_LIBRARIES})

#add_executable(occupancy_mapping src/occupancy_mapping.cpp src/ring_occupancy_grid.cpp)
#target_link_libraries(occupancy_mapping ${catkin_LIBRARIES})
#target_link_libraries(occupancy_mapping ${PCL_LIBRARIES})

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
# install(TARGETS particle_filter particle_filter_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_particle_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)









