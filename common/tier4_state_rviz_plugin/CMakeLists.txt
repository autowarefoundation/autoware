cmake_minimum_required(VERSION 3.14)
project(tier4_state_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/autoware_state_panel.cpp
  src/velocity_steering_factors_panel.cpp
  src/custom_toggle_switch.cpp
  src/custom_slider.cpp
  src/custom_container.cpp
  src/custom_button.cpp
  src/custom_icon_label.cpp
  src/custom_segmented_button.cpp
  src/custom_segmented_button_item.cpp
  src/custom_label.cpp
  src/include/material_colors.hpp
  src/include/autoware_state_panel.hpp
  src/include/custom_button.hpp
  src/include/custom_container.hpp
  src/include/custom_icon_label.hpp
  src/include/custom_label.hpp
  src/include/custom_segmented_button_item.hpp
  src/include/custom_segmented_button.hpp
  src/include/custom_slider.hpp
  src/include/custom_toggle_switch.hpp
  src/include/velocity_steering_factors_panel.hpp
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
)


target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)




ament_auto_package(
  INSTALL_TO_SHARE
  icons
  plugins
)
