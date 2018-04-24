cmake_minimum_required(VERSION 2.8.11)
project(amathutils_lib) # autoware math utility
find_package(catkin REQUIRED COMPONENTS
        roscpp
        autoware_msgs
        )

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

set(TARGET ${PROJECT_NAME})
set(TARGET_SRC src/Amathutils.cpp
        )

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
        ${TARGET_SRC}
        )
target_link_libraries(amathutils_lib
        ${CMAKE_THREAD_LIBS_INIT}
        )

add_dependencies(amathutils_lib
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS amathutils_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(DIRECTORY include/amathutils_lib
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )