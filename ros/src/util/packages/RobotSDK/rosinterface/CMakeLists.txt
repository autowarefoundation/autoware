cmake_minimum_required(VERSION 2.8.12)
project(rosinterface)

include(FindPkgConfig)
find_package(catkin REQUIRED
        autoware_build_flags
        roscpp
        tf
        )

set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rosinterface
        CATKIN_DEPENDS roscpp tf
)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)

qt5_wrap_cpp(rosinterface_SRC
        include/rosinterface/rosinterface.h
        )

add_library(rosinterface
        ROSInterface/rosinterface.cpp
        ${rosinterface_SRC}
        )

target_link_libraries(rosinterface
        ${catkin_LIBRARIES}
        Qt5::Core
        Qt5::Widgets
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


install(TARGETS rosinterface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
