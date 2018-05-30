cmake_minimum_required(VERSION 2.8.3)
project(javad_navsat_driver)

find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()
catkin_package()

install(PROGRAMS
   nodes/javad_serial_driver
   nodes/javad_topic_driver
   nodes/javad_topic_serial_reader
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Check package for pep8 style, add a test to fail on violations.
roslint_python()
#roslint_add_test()
