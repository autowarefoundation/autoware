cmake_minimum_required(VERSION 2.8.11)
project(amathutils_lib) # autoware math utility
find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        autoware_msgs
        )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES amathutils_lib
        CATKIN_DEPENDS roscpp autoware_msgs
)

include_directories(amathutils_lib
        include
        ${catkin_INCLUDE_DIRS}
        )

add_library(amathutils_lib
        src/Amathutils.cpp
        )
target_link_libraries(amathutils_lib
        ${catkin_LIBRARIES}
        )

add_dependencies(amathutils_lib
        ${catkin_EXPORTED_TARGETS}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )

install(TARGETS amathutils_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
