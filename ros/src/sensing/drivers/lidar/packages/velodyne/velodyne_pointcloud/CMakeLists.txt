cmake_minimum_required(VERSION 2.8.3)
project(velodyne_pointcloud)

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    velodyne_driver
    velodyne_msgs)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS} pcl_conversions)
find_package(Boost COMPONENTS signals)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})
             
generate_dynamic_reconfigure_options(
  cfg/VelodyneConfig.cfg)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    DEPENDS yaml-cpp python-yaml
    INCLUDE_DIRS include
    LIBRARIES velodyne_rawdata)
    
#add_executable(dynamic_reconfigure_node src/dynamic_reconfigure_node.cpp)
#target_link_libraries(dynamic_reconfigure_node
#   ${catkin_LIBRARIES}
# )

add_subdirectory(src/lib)
add_subdirectory(src/conversions)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
install(PROGRAMS scripts/gen_calibration.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  

if (CATKIN_ENABLE_TESTING)

  # these dependencies are only needed for unit testing
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
 
  # Download packet capture (PCAP) files containing test data.
  # Store them in devel-space, so rostest can easily find them.
  catkin_download_test_data(
    ${PROJECT_NAME}_tests_class.pcap
    http://download.ros.org/data/velodyne/class.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 65808d25772101358a3719b451b3d015)
  catkin_download_test_data(
    ${PROJECT_NAME}_tests_32e.pcap
    http://download.ros.org/data/velodyne/32e.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 e41d02aac34f0967c03a5597e1d554a9)
  catkin_download_test_data(
    ${PROJECT_NAME}_tests_64e_s2.1-300-sztaki.pcap
    http://download.ros.org/data/velodyne/64e_s2.1-300-sztaki.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 176c900ffb698f9b948a13e281ffc1a2)
  catkin_download_test_data(
    ${PROJECT_NAME}_tests_vlp16.pcap
    http://download.ros.org/data/velodyne/vlp16.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 f45c2bb1d7ee358274e423ea3b66fd73)

  # unit tests
  add_rostest(tests/cloud_node_hz.test)
  add_rostest(tests/cloud_nodelet_hz.test)
  add_rostest(tests/cloud_node_32e_hz.test)
  add_rostest(tests/cloud_nodelet_32e_hz.test)
  add_rostest(tests/cloud_node_64e_s2.1_hz.test)
  add_rostest(tests/cloud_nodelet_64e_s2.1_hz.test)
  add_rostest(tests/cloud_node_vlp16_hz.test)
  add_rostest(tests/cloud_nodelet_vlp16_hz.test)

  ## These tests don't work well enough to be worth the effort of
  ## running them:
  #add_rostest(tests/transform_node_hz.test)
  #add_rostest(tests/transform_nodelet_hz.test)
  
  # parse check all the launch/*.launch files
  roslaunch_add_file_check(launch)

endif (CATKIN_ENABLE_TESTING)
