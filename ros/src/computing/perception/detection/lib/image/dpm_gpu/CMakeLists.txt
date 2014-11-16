cmake_minimum_required(VERSION 2.8.3)
project(dpm_gpu)

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
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dpm_gpu
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_C_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/opencv2/legacy)
include_directories(/usr/local/cuda/include)
include_directories(src)

set(DPM_GPU_PATH src)
file(GLOB DPM_GPU_CXX_SOURCES "${DPM_GPU_PATH}/*.cpp")
file(GLOB DPM_GPU_CC_SOURCES "${DPM_GPU_PATH}/*.cc")

## Declare a cpp library
add_library(dpm_gpu
  ${DPM_GPU_CXX_SOURCES}
  ${DPM_GPU_CC_SOURCES}
  ${DPM_GPU_PATH}/conv.c
)

target_link_libraries(dpm_gpu
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  cuda
  )

set(CUDA_REG_CONF_OPTION "-Xptxas -v --maxrregcount 32")
set(CUDA_ARCH "sm_50") # Titan: sm_35 jetson: sm_32
set(CUDA_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${DPM_GPU_PATH}")

add_custom_command(
  OUTPUT GPU_function.cubin
  COMMAND
  "/usr/local/cuda/bin/nvcc" "-arch=${CUDA_ARCH}"
  "-o" "${CUBIN_OUTPUT}" "-cubin" "-Xptxas" "-v"
  "--maxrregcount" "32"
  "-I${CMAKE_CURRENT_SOURCE_DIR}/include" "-I${CUDA_FILE_PATH}"
  "${CMAKE_CURRENT_SOURCE_DIR}/${DPM_GPU_PATH}/GPU_function.cu"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${DPM_GPU_PATH}/GPU_function.cu"
)

add_custom_target(cuda_binary DEPENDS GPU_function.cubin)
