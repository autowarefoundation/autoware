cmake_minimum_required(VERSION 3.8)
project(autoware_mission_details_overlay_rviz_plugin)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(
  headers_to_moc
  include/mission_details_display.hpp
)

set(
  headers_to_not_moc
  include/remaining_distance_time_display.hpp
)

foreach(header "${headers_to_moc}")
  qt5_wrap_cpp(display_moc_files "${header}")
endforeach()

set(
  display_source_files
  src/overlay_utils.cpp
  src/mission_details_display.cpp
  src/remaining_distance_time_display.cpp
)

add_library(
  ${PROJECT_NAME} SHARED
  ${display_moc_files}
  ${headers_to_not_moc}
  ${display_source_files}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "${PROJECT_NAME}_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  rviz_common
  rviz_rendering
  autoware_internal_msgs
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rviz_common
  rviz_ogre_vendor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

# Copy the assets directory to the installation directory
install(
  DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

add_definitions(-DQT_NO_KEYWORDS)

ament_package(
  CONFIG_EXTRAS "autoware_mission_details_overlay_rviz_plugin-extras.cmake"
)
