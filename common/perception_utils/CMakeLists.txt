cmake_minimum_required(VERSION 3.14)
project(perception_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/run_length_encoder.cpp
)

find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

ament_auto_package()
