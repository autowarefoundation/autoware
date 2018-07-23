cmake_minimum_required(VERSION 2.8.3)
project(state_machine_lib)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        autoware_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES state_machine_lib
        CATKIN_DEPENDS roscpp autoware_msgs
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(state_machine_lib
        src/state_context.cpp
        )


target_link_libraries(state_machine_lib
        ${catkin_LIBRARIES}
        )

add_dependencies(${TARGET}
        ${catkin_EXPORTED_TARGETS}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )


install(TARGETS state_machine_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
