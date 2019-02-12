cmake_minimum_required(VERSION 2.8.3)
project(autoware_launcher_rviz)

find_package(catkin REQUIRED COMPONENTS rviz autoware_build_flags)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets Network)
set(QT_LIBRARIES Qt5::Widgets Qt5::Network)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS rviz
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

qt5_wrap_cpp(QT_HEADER_FILES
  src/module_panel.hpp
)

set(QT_SOURCE_FILES
  src/module_panel.cpp
)

add_library(${PROJECT_NAME}
  ${QT_HEADER_FILES}
  ${QT_SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
)

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autoware_launcher_rviz.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
