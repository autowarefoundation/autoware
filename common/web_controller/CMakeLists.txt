cmake_minimum_required(VERSION 3.5)
project(web_controller)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  www
  DESTINATION share/${PROJECT_NAME}/
)

ament_package(
)
