cmake_minimum_required(VERSION 2.8.3)
project(autoware_gnss_library)

find_package(catkin REQUIRED roscpp)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES autoware_gnss_library
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(autoware_gnss_library
        src/geo_pos_conv.cpp
        )

target_link_libraries(autoware_gnss_library
        ${catkin_LIBRARIES}
        )

install(
        DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

## Install executables and/or libraries
install(TARGETS autoware_gnss_library
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
