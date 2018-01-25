cmake_minimum_required(VERSION 2.8.3)
project(xsens_driver)

find_package(catkin REQUIRED COMPONENTS
  custom_msgs diagnostic_msgs geometry_msgs gps_common rospy sensor_msgs std_msgs tf
)

catkin_package(
    CATKIN_DEPENDS custom_msgs diagnostic_msgs geometry_msgs gps_common rospy sensor_msgs std_msgs tf
)

# add_library(test_py
#   src/${PROJECT_NAME}/test_py.cpp

# add_executable(test_py_node src/test_py_node.cpp)

# add_dependencies(test_py_node test_py_generate_messages_cpp)

# target_link_libraries(test_py_node

# install(TARGETS test_py test_py_node

#install(
#  DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

# catkin_add_gtest(${PROJECT_NAME}-test test/test_test_py.cpp)

catkin_install_python(PROGRAMS src/mtnode.py src/mtdevice.py src/mtdef.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
