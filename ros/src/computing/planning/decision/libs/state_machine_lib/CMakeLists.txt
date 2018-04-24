cmake_minimum_required(VERSION 2.8.3)
project(state_machine_lib)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        autoware_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES state_machine_lib
        CATKIN_DEPENDS roscpp autoware_msgs
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(TARGET ${PROJECT_NAME})
set(TARGET_SRC
        src/state_context.cpp
        )

add_library(${TARGET}
        ${TARGET_SRC}
        )


target_link_libraries(${TARGET}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_dependencies(${TARGET}
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS state_machine_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(DIRECTORY include/state_machine_lib
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )