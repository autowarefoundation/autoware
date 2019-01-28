cmake_minimum_required(VERSION 2.8.12)
project(points2image)

include(FindPkgConfig)

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        rosinterface
        fastvirtualscan
        cv_bridge
        tf
        pcl_ros
        pcl_conversions
        )

find_package(OpenCV REQUIRED)

set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(OpenGL REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp
        std_msgs
        sensor_msgs
        autoware_msgs
        rosinterface
        fastvirtualscan
        cv_bridge
        tf
        pcl_ros
        pcl_conversions
)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

# library
add_library(points_image
        lib/points_image/points_image.cpp
        )
add_dependencies(points_image
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )
target_link_libraries(points_image
        ${OpenCV_LIBS}
        )

qt5_wrap_ui(points2vscan_ui_mainwindow nodes/points2vscan/mainwindow.ui)

# points2vscan
add_executable(points2vscan
        nodes/points2vscan/main.cpp
        nodes/points2vscan/mainwindow.cpp
        nodes/points2vscan/mainwindow.h
        ${points2vscan_ui_mainwindow}
        )

target_link_libraries(points2vscan
        ${catkin_LIBRARIES}
        Qt5::Core
        Qt5::Widgets
        points_image
        ${OPENGL_LIBRARIES}
        ${OpenCV_LIBS}
        )
add_dependencies(points2vscan
        ${catkin_EXPORTED_TARGETS}
        )

# points2image
add_executable(points2image nodes/points2image/points2image.cpp)
target_link_libraries(points2image
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        points_image
        )
add_dependencies(points2image
        ${catkin_EXPORTED_TARGETS}
        )

# vscan2image
add_executable(vscan2image nodes/vscan2image/vscan2image.cpp)
target_link_libraries(vscan2image
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        points_image
        )
add_dependencies(vscan2image
        ${catkin_EXPORTED_TARGETS}
        )


# vscan2linelist
add_executable(vscan2linelist nodes/vscan2linelist/vscan2linelist.cpp)
target_link_libraries(vscan2linelist
        ${catkin_LIBRARIES}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )

install(TARGETS points_image vscan2linelist vscan2image points2image points2vscan
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
