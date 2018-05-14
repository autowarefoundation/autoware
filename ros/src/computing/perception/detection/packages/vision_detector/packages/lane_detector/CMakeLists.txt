cmake_minimum_required(VERSION 2.8.3)
project(lane_detector)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        autoware_msgs
        )
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES lane_detector
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        nodes/lane_detector
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_detector
        nodes/lane_detector/lane_detector.cpp
        )

target_link_libraries(lane_detector
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
add_dependencies(lane_detector
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS lane_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)