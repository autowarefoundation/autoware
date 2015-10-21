cmake_minimum_required(VERSION 2.8.3)
project(requirements_version_checker)

execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if (NOT "${ROS_VERSION}" MATCHES "(indigo|jade)")
message(
  FATAL_ERROR
  "You must need ROS indigo or jade(Your ROS version is '${ROS_VERSION}')"
)
endif()

execute_process(
  COMMAND pkg-config --modversion opencv
  OUTPUT_VARIABLE OPENCV_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if ("${OPENCV_VERSION}" VERSION_LESS "2.4.8")
message(
  FATAL_ERROR
  "You must need OpenCV 2.4.8 or higher(Your version is '${OPENCV_VERSION})'"
)
endif()
