# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 2.8.7)
project(integrated_viewer)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  runtime_manager
  points2image
  rviz
  )

set(CMAKE_CXX_FLAGS
  "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}"
  )

catkin_package(
  DEPENDS rviz
  )

# We specify Qt4 will be used as RViz using them and 
# Libraries conflict will be happen if we specify Qt5.
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

include_directories(
  lib/
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

# I prefer the Qt signals and slots to avoid defining "emit", "slots",
# etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

# Here we wpecify which ui files need to be run through "uic",
# Qt's user-interface compiler.
qt4_wrap_ui(UI_HEADERS
  node/traffic_light_plugin/form.ui
  node/image_viewer_plugin/image_viewer_form.ui
  )

# Here we specify which header files need to be run through "moc",
# Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  node/traffic_light_plugin/traffic_light_plugin.h
  node/image_viewer_plugin/image_viewer_plugin.h
  )

# Here we specify the list of source files, including the output of
# the previous command which is stored in ``${MOC_FILES}`` and ``${UI_HEADERS}``.
set(SOURCE_FILES
  lib/convert_image.h
  node/traffic_light_plugin/traffic_light_plugin.cpp
  node/image_viewer_plugin/image_viewer_plugin.cpp
  node/image_viewer_plugin/draw_rects.cpp
  node/image_viewer_plugin/draw_points.cpp
  ${MOC_FILES}
  ${UI_HEADERS})

# An rviz plugin is just a shared library, so here we declare the
# library to be called ``${PROJECT_NAME}`` (which is
# "rviz_plugin_tutorials", or whatever your version of this project
# is called) and specify the list of source files we collected above
# in ``${SOURCE_FILES}``.
# 
add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
  COMPILE_FLAGS
  "-DIMAGE_VIEWER_DEFAULT_IMAGE=${CMAKE_CURRENT_SOURCE_DIR}/node/image_viewer_plugin/autoware_logo.png"
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES})


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_dependencies(${PROJECT_NAME} 
  runtime_manager_generate_messages_cpp
  cv_tracker_generate_messages_cpp
  points2image_generate_mssages_cpp)