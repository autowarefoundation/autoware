cmake_minimum_required(VERSION 2.8.3)
project(pedestrian_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
   CATKIN_DEPENDS car_detector
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/opencv2/legacy)

add_executable(pedestrian_dpm
  nodes/pedestrian_dpm/pedestrian_dpm.cpp
)

target_link_libraries(pedestrian_dpm
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  dpm
)

if(EXISTS "/usr/local/cuda")
include_directories(/usr/local/cuda/include)

add_executable(pedestrian_dpm_gpu
  nodes/pedestrian_dpm_gpu/pedestrian_dpm_gpu.cpp
)

SET_TARGET_PROPERTIES(pedestrian_dpm_gpu
  PROPERTIES
  COMPILE_FLAGS "-DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin"
)

target_link_libraries(pedestrian_dpm_gpu
  ${catkin_LIBRARIES}
  cuda
  dpm_gpu
)
else()
message("CUDA is not installed. 'pedestrian_dpm_gpu' is not built.")
endif()

include_directories(lib/fusion)
add_library(fusion
  lib/fusion/showboxes.cpp
)

## Declare a cpp executable
add_executable(pedestrian_fusion
  nodes/pedestrian_fusion/pedestrian_fusion.cpp  nodes/pedestrian_fusion/fusion.cpp
)

target_link_libraries(pedestrian_fusion
  ${catkin_LIBRARIES} ${OpenCV_LIBS} fusion
)

add_executable(pedestrian_tracking
  nodes/pedestrian_tracking/pedestrian_tracking.cpp
)
