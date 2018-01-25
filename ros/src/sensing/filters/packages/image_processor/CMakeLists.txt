cmake_minimum_required(VERSION 2.8.3)
project(image_processor)


set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    std_msgs
    sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS cv_bridge roscpp sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
)

#Image rotator
add_executable(image_rotator
    nodes/image_rotator/image_rotator_node.cpp
)
target_link_libraries(image_rotator
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
add_dependencies(image_rotator ${catkin_EXPORTED_TARGETS})

#Image rectifier
add_executable(image_rectifier
    nodes/image_rectifier/image_rectifier_node.cpp
)
target_link_libraries(image_rectifier
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
add_dependencies(image_rectifier ${catkin_EXPORTED_TARGETS})


install(TARGETS image_rotator image_rectifier
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
