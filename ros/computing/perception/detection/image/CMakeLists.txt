cmake_minimum_required(VERSION 2.8.3)
project(image)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ImageObjects.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

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
  LIBRARIES image
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/opencv2/legacy)
include_directories(/usr/local/cuda/include)

file(GLOB DPM_OCV_CXX_SOURCES "detectors/dpm_ocv/src/*.cpp")

set(DPM_GPU_PATH detectors/dpm_gpu/src)
file(GLOB DPM_GPU_CXX_SOURCES "${DPM_GPU_PATH}/*.cpp")
file(GLOB DPM_GPU_CC_SOURCES "${DPM_GPU_PATH}/*.cc")

set(DPM_GPU_SOURCES
  ${DPM_GPU_CXX_SOURCES}
  ${DPM_GPU_CC_SOURCES}
  ${DPM_GPU_PATH}/conv.c
)

add_executable(image_detector
 image_detector.cpp
 ${DPM_OCV_CXX_SOURCES}
 ${DPM_GPU_SOURCES}
)

target_link_libraries(image_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  cuda
)
add_dependencies(image_detector image_generate_messages_cpp)

set(CUDA_REG_CONF_OPTION "-Xptxas -v --maxrregcount 32")
set(CUDA_ARCH "sm_50") # Titan: sm_35 jetson: sm_32

## XXX How to generate .cubin file with CMake commands
set(CUBIN_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR})
set(CUDA_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${DPM_GPU_PATH}")
set(CUDA_FILE_OUTPUT "${CATKIN_DEVEL_PREFIX}/lib/image")

execute_process(
  COMMAND "/usr/local/cuda/bin/nvcc" "-arch=${CUDA_ARCH}"
          "-o" "${CUDA_FILE_OUTPUT}/GPU_function.cubin"
          "-cubin" "-Xptxas" "-v" "--maxrregcount" "32"
          "-I${CMAKE_CURRENT_SOURCE_DIR}/include" "-I${CUDA_FILE_PATH}"
          "${CMAKE_CURRENT_SOURCE_DIR}/${DPM_GPU_PATH}/GPU_function.cu"
)
