# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(posedetection_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge message_generation message_filters)

include_directories(SYSTEM include
                           ${catkin_INCLUDE_DIRS})

add_message_files(FILES
  Curve1D.msg
  Feature0D.msg
  Feature1D.msg
  ImageFeature0D.msg
  ImageFeature1D.msg
  Object6DPose.msg
  ObjectDetection.msg
)

add_service_files(FILES
  Detect.srv
  Feature0DDetect.srv
  Feature1DDetect.srv
  TargetObj.srv
)

# to compatible with fuerte directories
generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime
    DEPENDS
    INCLUDE_DIRS include
    LIBRARIES # TODO
)

add_executable(feature0d_view src/feature0d_view.cpp)
target_link_libraries(feature0d_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(feature0d_view posedetection_msgs_gencpp)
add_executable(feature0d_to_image src/feature0d_to_image.cpp)
target_link_libraries(feature0d_to_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(feature0d_to_image posedetection_msgs_gencpp)

install(TARGETS feature0d_view feature0d_to_image
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
