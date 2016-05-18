# these plugins should be uploaded to upstream repository
# https://github.com/ros-visualization/rviz/pull/634

# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_rviz_plugins)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rviz jsk_hark_msgs jsk_footstep_msgs jsk_recognition_utils
  #people_msgs
  message_generation std_msgs diagnostic_msgs cv_bridge
  jsk_topic_tools
  image_geometry
  #view_controller_msgs
  #roseus
  geometry_msgs
  jsk_recognition_msgs)

add_message_files(FILES
  OverlayText.msg OverlayMenu.msg TransformableMarkerOperate.msg ObjectFitCommand.msg
  Pictogram.msg PictogramArray.msg RecordCommand.msg)
add_service_files(FILES
  RequestMarkerOperate.srv EusCommand.srv
  Screenshot.srv
  )
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    DEPENDS rviz
    CATKIN_DEPENDS jsk_hark_msgs jsk_footstep_msgs
    jsk_recognition_utils cv_bridge people_msgs image_geometry
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


set(ROS_BUILD_TYPE Release)

include_directories(src ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS -g)


#find_package(wxWidgets REQUIRED)
#include(${wxWidgets_USE_FILE})
#include_directories( ${wxWidgets_INCLUDE_DIRS} )

include_directories(${CMAKE_CURRENT_BINARY_DIR})

#set(SOURCE_FILES src/ambient_sound_display.cpp src/ambient_sound_visual.cpp)
qt4_wrap_cpp(MOC_FILES
  src/ambient_sound_display_groovy.h
  src/select_point_cloud_publish_action.h
  src/footstep_display.h
  src/publish_topic.h
  src/cancel_action.h
  src/record_action.h
  src/polygon_array_display.h
  src/normal_display.h
  src/overlay_text_display.h
  src/overlay_menu_display.h
  src/overlay_image_display.h
  src/overlay_camera_display.h
  src/plotter_2d_display.h
  src/pie_chart_display.h
  src/diagnostics_display.h
  src/quiet_interactive_marker_display.h
  src/bounding_box_array_display.h
  src/torus_array_display.h
  src/overlay_diagnostic_display.h
  src/target_visualizer_display.h
  #src/people_position_measurement_array_display.h
  src/simple_occupancy_grid_array_display.h
  src/camera_info_display.h
  src/close_all_tool.h
  src/open_all_tool.h
  src/transformable_marker_operator.h
  src/object_fit_operator.h
  src/robot_command_interface.h
  src/empty_service_call_interface.h
  src/pictogram_display.h
  src/pictogram_array_display.h
  #src/view_controller/tablet_view_controller.h
  #src/tablet_controller_panel.h
  src/video_capture_display.h
  src/twist_stamped_display.h
  src/pose_array_display.h
  src/tf_trajectory_display.h
)

set(SOURCE_FILES
  src/ambient_sound_display_groovy.cpp
  src/ambient_sound_visual.cpp
  src/footstep_display.cpp
  src/publish_topic.cpp
  src/cancel_action.cpp
  src/record_action.cpp
  src/select_point_cloud_publish_action.cpp
  src/polygon_array_display.cpp
  src/normal_display.cpp
  src/normal_visual.cpp
  src/overlay_text_display.cpp
  src/overlay_menu_display.cpp
  src/overlay_image_display.cpp
  src/overlay_camera_display.cpp
  src/plotter_2d_display.cpp
  src/pie_chart_display.cpp
  src/diagnostics_display.cpp
  src/bounding_box_array_display.cpp
  src/torus_array_display.cpp
  src/quiet_interactive_marker_display.cpp
  src/target_visualizer_display.cpp
  src/overlay_diagnostic_display.cpp
  src/simple_occupancy_grid_array_display.cpp
  #src/people_position_measurement_array_display.cpp
  src/overlay_utils.cpp
  src/facing_visualizer.cpp
  src/camera_info_display.cpp
  src/close_all_tool.cpp
  src/open_all_tool.cpp
  src/screenshot_listener_tool.cpp
  src/transformable_marker_operator.cpp
  src/object_fit_operator.cpp
  src/robot_command_interface.cpp
  src/empty_service_call_interface.cpp
  src/pictogram_display.cpp
  src/pictogram_array_display.cpp
  #src/view_controller/tablet_view_controller.cpp
  #src/tablet_controller_panel.cpp
  src/video_capture_display.cpp
  src/twist_stamped_display.cpp
  src/pose_array_display.cpp
  src/tf_trajectory_display.cpp
  ${MOC_FILES}
)

add_library(jsk_rviz_plugins ${SOURCE_FILES} ${UIC_FILES})
target_link_libraries(jsk_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(jsk_rviz_plugins jsk_hark_msgs_gencpp ${PROJECT_NAME}_gencpp)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS jsk_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY scripts launch cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
install(DIRECTORY ./src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
