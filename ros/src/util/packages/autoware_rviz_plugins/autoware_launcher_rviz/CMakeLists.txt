cmake_minimum_required(VERSION 2.8.3)
project(autoware_launcher_rviz)

find_package(catkin REQUIRED COMPONENTS rosconsole rviz autoware_build_flags)
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

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
)

