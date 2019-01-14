cmake_minimum_required(VERSION 2.8.3)
project(fusion)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        cv_bridge
        autoware_msgs
        )
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES fusion
        CATKIN_DEPENDS roscpp autoware_msgs cv_bridge
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(fusion
        src/fusion.cpp
        src/search_distance.cpp
        )

target_link_libraries(fusion
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )
add_dependencies(fusion
        ${catkin_EXPORTED_TARGETS}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(TARGETS
        fusion
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
