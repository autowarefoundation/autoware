cmake_minimum_required(VERSION 3.14)
project(tier4_traffic_light_rviz_plugin)

find_package(autoware_cmake REQUIRED)
# TODO(Mamoru Sobue): workaround to avoid 'missing tinyxml2::tinyxml2'
# From tinyxml2_vendor/cmake/Modules/FindTinyXML2.cmake
find_package(TinyXML2 CONFIG QUIET)
if(NOT TinyXML2_FOUND)
  find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
  find_library(TINYXML2_LIBRARY tinyxml2)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TINYXML2_LIBRARY TINYXML2_INCLUDE_DIR)
  mark_as_advanced(TINYXML2_INCLUDE_DIR TINYXML2_LIBRARY)
  if(NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY IMPORTED_LOCATION ${TINYXML2_LIBRARY})
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${TINYXML2_INCLUDE_DIR})
    list(APPEND TinyXML2_TARGETS tinyxml2::tinyxml2)
  endif()
endif()

autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_publish_panel.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC -Wno-error=deprecated-copy -Wno-error=pedantic)
# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  plugins
)
