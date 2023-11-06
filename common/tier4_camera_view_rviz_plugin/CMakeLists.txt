cmake_minimum_required(VERSION 3.14)
project(tier4_camera_view_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/third_person_view_controller.cpp
  src/third_person_view_tool.cpp
  src/bird_eye_view_tool.cpp
  src/bird_eye_view_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(INSTALL_TO_SHARE icons)
