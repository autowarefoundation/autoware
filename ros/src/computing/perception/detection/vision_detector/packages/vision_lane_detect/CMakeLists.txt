cmake_minimum_required(VERSION 2.8.3)
project(vision_lane_detect)


find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        )
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES vision_lane_detect
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        nodes/vision_lane_detect
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(vision_lane_detect
        nodes/vision_lane_detect/vision_lane_detect.cpp
        )

target_link_libraries(vision_lane_detect
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(vision_lane_detect
        ${catkin_EXPORTED_TARGETS}
        autoware_msgs_generate_messages_cpp
        )

install(TARGETS vision_lane_detect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
