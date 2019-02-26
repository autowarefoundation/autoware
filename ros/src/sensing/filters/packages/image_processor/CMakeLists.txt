cmake_minimum_required(VERSION 2.8.3)
project(image_processor)


find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        cv_bridge
        )

catkin_package(CATKIN_DEPENDS
        sensor_msgs
        cv_bridge
        )

find_package(OpenCV REQUIRED)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

#Image rotator
add_executable(image_rotator
        nodes/image_rotator/image_rotator_node.cpp
        )

target_include_directories(image_rotator PRIVATE
        ${OpenCV_INCLUDE_DIR}
        )

target_link_libraries(image_rotator
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

#Image rectifier
add_executable(image_rectifier
        nodes/image_rectifier/image_rectifier_node.cpp
        )

target_include_directories(image_rectifier PRIVATE
        ${OpenCV_INCLUDE_DIR}
        )

target_link_libraries(image_rectifier
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

#Image Stitcher

#Image Blender

#############
## Install ##
#############

install(TARGETS image_rotator image_rectifier
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
