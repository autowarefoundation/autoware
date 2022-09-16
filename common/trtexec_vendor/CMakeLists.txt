cmake_minimum_required(VERSION 3.14)
project(trtexec_vendor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(CUDA)
find_package(cudnn_cmake_module REQUIRED)
find_package(CUDNN)
find_package(tensorrt_cmake_module REQUIRED)
find_package(TENSORRT)

if(NOT (${CUDA_FOUND} AND ${CUDNN_FOUND} AND ${TENSORRT_FOUND}))
  message(WARNING "cuda, cudnn, tensorrt libraries are not found")
  return()
endif()

if(${TENSORRT_VERSION} VERSION_LESS 8.2.1)
  message(WARNING "The tensorrt version less than 8.2.1 isn't supported.")
  return()
endif()

set(TRTEXEC_DEFAULT_BIN /usr/src/tensorrt/bin/trtexec)
if(NOT EXISTS TRTEXEC_DEFAULT_BIN)
  include(FetchContent)
  if(${TENSORRT_VERSION} VERSION_EQUAL 8.4.2)
    set(TENSORRT_VERSION 8.4.1)
  elseif(${TENSORRT_VERSION} VERSION_LESS_EQUAL 8.2.5 AND ${TENSORRT_VERSION} VERSION_GREATER 8.2.1)
    set(TENSORRT_VERSION 8.2.1)
  endif()
  fetchcontent_declare(tensorrt
    GIT_REPOSITORY https://github.com/NVIDIA/TensorRT
    GIT_TAG ${TENSORRT_VERSION}
    GIT_SUBMODULES ""
  )
  fetchcontent_getproperties(tensorrt)
  if(NOT tensorrt_POPULATED)
    fetchcontent_populate(tensorrt)
  endif()
  set(TRTEXEC_SOURCES
    ${tensorrt_SOURCE_DIR}/samples/trtexec/trtexec.cpp
    ${tensorrt_SOURCE_DIR}/samples/common/sampleEngines.cpp
    ${tensorrt_SOURCE_DIR}/samples/common/sampleInference.cpp
    ${tensorrt_SOURCE_DIR}/samples/common/sampleOptions.cpp
    ${tensorrt_SOURCE_DIR}/samples/common/sampleReporting.cpp
    ${tensorrt_SOURCE_DIR}/samples/common/logger.cpp
  )
  if(${TENSORRT_VERSION} VERSION_GREATER_EQUAL 8.4)
    list(APPEND TRTEXEC_SOURCES
      ${tensorrt_SOURCE_DIR}/samples/common/sampleUtils.cpp
    )
  endif()
  cuda_add_executable(${PROJECT_NAME}
    ${TRTEXEC_SOURCES}
  )
  target_link_libraries(${PROJECT_NAME}
    ${TENSORRT_LIBRARIES}
  )
  target_include_directories(${PROJECT_NAME}
    PRIVATE ${tensorrt_SOURCE_DIR}/samples/common
  )

  set_target_properties(${PROJECT_NAME}
    PROPERTIES OUTPUT_NAME trtexec
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

ament_package()
