cmake_minimum_required(VERSION 3.5)
project(accel_brake_map_calibrator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(accel_brake_map_calibrator
  src/accel_brake_map_calibrator_node.cpp
  src/main.cpp
)
ament_target_dependencies(accel_brake_map_calibrator)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(accel_brake_map_calibrator PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

install(
  PROGRAMS
  scripts/__init__.py
  scripts/config.py
  scripts/delay_estimator.py
  scripts/plotter.py
  scripts/view_statistics.py
  scripts/calc_utils.py
  scripts/csv_reader.py
  scripts/log_analyzer.py
  scripts/view_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  rviz)
