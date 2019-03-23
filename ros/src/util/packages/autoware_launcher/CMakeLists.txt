cmake_minimum_required(VERSION 2.8.3)
project(autoware_launcher)

find_package(catkin REQUIRED COMPONENTS python_qt_binding)
catkin_python_setup()

catkin_package(
  LIBRARIES autoware_launcher
  CATKIN_DEPENDS python_qt_binding
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()

catkin_install_python(
  PROGRAMS scripts/main scripts/tool
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
)

install(DIRECTORY plugins/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
)

install(DIRECTORY profiles/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/profiles
)
