cmake_minimum_required(VERSION 2.8.3)
project(deal_image)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ../../include
)

add_library(deal_image deal_image.c)
