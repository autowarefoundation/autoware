# these plugins should be uploaded to upstream repository
# https://github.com/ros-visualization/rviz/pull/634

# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_rviz_plugins)
add_compile_options(-std=c++11)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rviz jsk_hark_msgs jsk_footstep_msgs jsk_recognition_utils
  people_msgs
  message_generation std_msgs diagnostic_msgs cv_bridge
  jsk_topic_tools
  image_geometry
  view_controller_msgs
  geometry_msgs
  dynamic_reconfigure
  jsk_recognition_msgs
  jsk_gui_msgs)

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

add_message_files(FILES
  OverlayText.msg OverlayMenu.msg TransformableMarkerOperate.msg ObjectFitCommand.msg
  Pictogram.msg PictogramArray.msg RecordCommand.msg StringStamped.msg)
add_service_files(FILES
  RequestMarkerOperate.srv EusCommand.srv
  Screenshot.srv
  )
catkin_python_setup()
generate_dynamic_reconfigure_options(
  cfg/ClassificationResultVisualizer.cfg
  cfg/ContactStateMarker.cfg
  cfg/OverlayTextInterface.cfg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    DEPENDS rviz
    CATKIN_DEPENDS jsk_hark_msgs jsk_footstep_msgs
    jsk_recognition_utils cv_bridge people_msgs image_geometry
    INCLUDE_DIRS # TODO include
    LIBRARIES ${PROJECT_NAME}
)


set(ROS_BUILD_TYPE Release)

include_directories(src ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
else()
  find_package(Qt5Widgets REQUIRED)
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

#set(SOURCE_FILES src/ambient_sound_display.cpp src/ambient_sound_visual.cpp)

set(HEADER_FILES
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
  src/bounding_box_display.h
  src/bounding_box_array_display.h
  src/torus_array_display.h
  src/overlay_diagnostic_display.h
  src/target_visualizer_display.h
  src/people_position_measurement_array_display.h
  src/simple_occupancy_grid_array_display.h
  src/camera_info_display.h
  src/close_all_tool.h
  src/open_all_tool.h
  src/overlay_picker_tool.h
  src/object_fit_operator.h
  src/robot_command_interface.h
  src/empty_service_call_interface.h
  src/pictogram_display.h
  src/pictogram_array_display.h
  src/view_controller/tablet_view_controller.h
  src/tablet_controller_panel.h
  src/video_capture_display.h
  src/twist_stamped_display.h
  src/pose_array_display.h
  src/tf_trajectory_display.h
  src/yes_no_button_interface.h
  src/segment_array_display.h
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

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
  src/bounding_box_display.cpp
  src/bounding_box_array_display.cpp
  src/torus_array_display.cpp
  src/quiet_interactive_marker_display.cpp
  src/target_visualizer_display.cpp
  src/overlay_diagnostic_display.cpp
  src/simple_occupancy_grid_array_display.cpp
  src/people_position_measurement_array_display.cpp
  src/overlay_utils.cpp
  src/facing_visualizer.cpp
  src/camera_info_display.cpp
  src/close_all_tool.cpp
  src/open_all_tool.cpp
  src/overlay_picker_tool.cpp
  src/screenshot_listener_tool.cpp
  src/object_fit_operator.cpp
  src/robot_command_interface.cpp
  src/empty_service_call_interface.cpp
  src/pictogram_display.cpp
  src/pictogram_array_display.cpp
  src/view_controller/tablet_view_controller.cpp
  src/tablet_controller_panel.cpp
  src/video_capture_display.cpp
  src/twist_stamped_display.cpp
  src/pose_array_display.cpp
  src/tf_trajectory_display.cpp
  src/yes_no_button_interface.cpp
  src/segment_array_display.cpp
  ${MOC_FILES}
)

add_library(jsk_rviz_plugins ${SOURCE_FILES} ${UIC_FILES})
if(rviz_QT_VERSION VERSION_LESS "5")
  target_link_libraries(jsk_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
else()
  target_link_libraries(jsk_rviz_plugins Qt5::Widgets ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
endif()
add_dependencies(jsk_rviz_plugins
  jsk_footstep_msgs_generate_messages_cpp
  jsk_gui_msgs_generate_messages_cpp
  jsk_hark_msgs_generate_messages_cpp
  jsk_recognition_msgs_generate_messages_cpp
  people_msgs_generate_messages_cpp
  view_controller_msgs_generate_messages_cpp
  ${PROJECT_NAME}_generate_messages_cpp)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(jsk_rviz_plugins PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()

# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(TARGETS jsk_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY cfg config icons launch samples test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
