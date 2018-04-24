cmake_minimum_required(VERSION 2.8.3)
project(gnss)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES gnss
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(gnss
        src/geo_pos_conv.cpp
        )

target_link_libraries(gnss
        ${catkin_LIBRARIES}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )


install(TARGETS gnss
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
