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

if(EXISTS "/usr/local/cuda")
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

set(CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
set(CUDA_CAPABILITY_VERSION_CHECKER
  "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker"
)

execute_process(
  COMMAND
  ${CUDA_COMPILER}
  "-o" ${CUDA_CAPABILITY_VERSION_CHECKER}
  "${CMAKE_CURRENT_SOURCE_DIR}/util/capability_version_checker.cpp"
)

execute_process(
  COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
  OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if("${CUDA_CAPABILITY_VERSION}" STREQUAL "")
message(FATAL_ERROR "Error: Missing CUDA capability version")
endif()

set(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
set(CUDA_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_custom_command(
  OUTPUT GPU_function.cubin
  COMMAND
  ${CUDA_COMPILER} "-arch=${CUDA_ARCH}"
  "-o" "${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin"
  "-cubin" "-Xptxas" "-v" "--maxrregcount" "32" "-I${CUDA_FILE_PATH}"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/GPU_function.cu"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/GPU_function.cu"
)

add_custom_target(cuda_binary DEPENDS GPU_function.cubin)
add_dependencies(dpm_gpu cuda_binary)

else()
message("CUDA is not installed. 'libdpm_gpu' is not built.")
endif()
