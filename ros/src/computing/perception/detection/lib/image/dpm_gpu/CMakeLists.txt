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
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_C_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/opencv2/legacy)
include_directories(/usr/local/cuda/include)
include_directories(src)

file(GLOB DPM_GPU_CXX_SOURCES "src/*.cpp")
file(GLOB DPM_GPU_CC_SOURCES "src/*.cc")

set(DPM_GPU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/models_TTIC/")

## Declare a cpp library
add_library(dpm_gpu
  ${DPM_GPU_CXX_SOURCES}
  ${DPM_GPU_CC_SOURCES}
  src/conv.c
)

SET_TARGET_PROPERTIES(dpm_gpu
  PROPERTIES COMPILE_FLAGS
  "-DDPM_GPU_ROOT=${DPM_GPU_ROOT}"
)

target_link_libraries(dpm_gpu
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  cuda
  )

execute_process(
  COMMAND lspci
  COMMAND grep "GTX 750"
  OUTPUT_VARIABLE CUDA_GPU_TYPE
)

if("${CUDA_GPU_TYPE}" STREQUAL "")
set(CUDA_ARCH "sm_30")
else(${CUDA_GPU_TYPE})
set(CUDA_ARCH "sm_50")
endif(${CUDA_GPU_TYPE})

set(CUDA_REG_CONF_OPTION "-Xptxas -v --maxrregcount 32")
set(CUDA_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_custom_command(
  OUTPUT GPU_function.cubin
  COMMAND
  "/usr/local/cuda/bin/nvcc" "-arch=${CUDA_ARCH}"
  "-o" "${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin"
  "-cubin" "-Xptxas" "-v" "--maxrregcount" "32" "-I${CUDA_FILE_PATH}"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/GPU_function.cu"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/GPU_function.cu"
)

add_custom_target(cuda_binary DEPENDS GPU_function.cubin)
add_dependencies(dpm_gpu cuda_binary)
