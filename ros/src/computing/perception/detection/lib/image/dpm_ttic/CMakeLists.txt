cmake_minimum_required(VERSION 2.8.3)
project(libdpm_ttic)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  message_generation
)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(CUDA)

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

catkin_package(
  INCLUDE_DIRS include/dpm_ttic
  LIBRARIES libdpm_ttic
)

SET(CMAKE_C_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

IF(CUDA_FOUND)
INCLUDE_DIRECTORIES(
  include/dpm_ttic
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  "common"
  "cpu"
  "gpu"
)

IF("${ARCHITECTURE}" MATCHES "^arm")
LINK_DIRECTORIES(/usr/lib/arm-linux-gnueabihf/tegra)
ENDIF()

FILE(GLOB DPM_TTIC_COMMON_IMPL_SOURCES "common/*.cpp")
FILE(GLOB DPM_TTIC_CPU_IMPL_SOURCES "cpu/*.cpp")
FILE(GLOB DPM_TTIC_GPU_IMPL_SOURCES "gpu/*.cpp")

## Declare a cpp library
ADD_LIBRARY(libdpm_ttic
  ${DPM_TTIC_COMMON_IMPL_SOURCES}
  ${DPM_TTIC_CPU_IMPL_SOURCES}
  ${DPM_TTIC_GPU_IMPL_SOURCES}
)
SET_TARGET_PROPERTIES(libdpm_ttic PROPERTIES OUTPUT_NAME dpm_ttic)

TARGET_LINK_LIBRARIES(libdpm_ttic
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  cuda
)

SET(CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")

IF("${ARCHITECTURE}" MATCHES "^arm")
SET(CUDA_LDFLAGS -L/usr/lib/arm-linux-gnueabihf/tegra -lcuda)
ELSE()
SET(CUDA_LDFLAGS -lcuda)
ENDIF()

SET(CUDA_CAPABILITY_VERSION_CHECKER
  "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker"
)

EXECUTE_PROCESS(
  COMMAND
  ${CUDA_COMPILER}
  -o ${CUDA_CAPABILITY_VERSION_CHECKER}
  "${CMAKE_CURRENT_SOURCE_DIR}/util/capability_version_checker.cpp"
  ${CUDA_LDFLAGS}
)

EXECUTE_PROCESS(
  COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
  OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

IF("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
SET(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
ELSE()
SET(CUDA_ARCH "sm_52")
ENDIF()

SET(CUDA_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/gpu")
ADD_CUSTOM_COMMAND(
  OUTPUT GPU_function.cubin
  COMMAND
  ${CUDA_COMPILER} "-arch=${CUDA_ARCH}"
  "-o" "${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin"
  "-cubin" "-Xptxas" "-v" "--maxrregcount" "32" "-I${CUDA_FILE_PATH}"
  "${CMAKE_CURRENT_SOURCE_DIR}/gpu/GPU_function.cu"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/gpu/GPU_function.cu"
)

ADD_CUSTOM_TARGET(cuda_binary DEPENDS GPU_function.cubin)
ADD_DEPENDENCIES(libdpm_ttic
  cuda_binary
  runtime_manager_generate_messages_cpp)

ELSE()

INCLUDE_DIRECTORIES(
  include/dpm_ttic
  ${catkin_INCLUDE_DIRS}
  "common"
  "cpu"
)

FILE(GLOB DPM_TTIC_COMMON_IMPL_SOURCES "common/*.cpp")
FILE(GLOB DPM_TTIC_CPU_IMPL_SOURCES "cpu/*.cpp")

## Declare a cpp library
ADD_LIBRARY(libdpm_ttic
  ${DPM_TTIC_COMMON_IMPL_SOURCES}
  ${DPM_TTIC_CPU_IMPL_SOURCES}
)
SET_TARGET_PROPERTIES(libdpm_ttic PROPERTIES OUTPUT_NAME dpm_ttic)

TARGET_LINK_LIBRARIES(libdpm_ttic
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

ADD_DEPENDENCIES(libdpm_ttic
  runtime_manager_generate_messages_cpp)

ENDIF()

#############
## Install ##
#############

INSTALL(
  DIRECTORY include/dpm_ttic
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
