cmake_minimum_required(VERSION 3.5)
project(tensorrt_yolox)

find_package(tensorrt_common)
if(NOT ${tensorrt_common_FOUND})
  message(WARNING "The tensorrt_common package is not found. Please check its dependencies.")
  return()
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(OpenMP)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

##########
# Download pretrained model
set(DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if(NOT EXISTS "${DATA_PATH}")
  execute_process(COMMAND mkdir -p ${DATA_PATH})
endif()
function(download FILE_NAME FILE_HASH)
  message(STATUS "Checking and downloading ${FILE_NAME}")
  set(FILE_PATH ${DATA_PATH}/${FILE_NAME})
  set(STATUS_CODE 0)
  message(STATUS "start ${FILE_NAME}")
  if(EXISTS ${FILE_PATH})
    message(STATUS "found ${FILE_NAME}")
    file(MD5 ${FILE_PATH} EXISTING_FILE_HASH)
    if(${FILE_HASH} STREQUAL ${EXISTING_FILE_HASH})
      message(STATUS "same ${FILE_NAME}")
      message(STATUS "File already exists.")
    else()
      message(STATUS "diff ${FILE_NAME}")
      message(STATUS "File hash changes. Downloading now ...")
      file(DOWNLOAD https://awf.ml.dev.web.auto/perception/models/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 3600)
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    endif()
  else()
    message(STATUS "not found ${FILE_NAME}")
    message(STATUS "File doesn't exists. Downloading now ...")
    file(DOWNLOAD https://awf.ml.dev.web.auto/perception/models/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 3600)
    list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
    list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
  endif()
  if(${STATUS_CODE} EQUAL 0)
    message(STATUS "Download completed successfully!")
  else()
    message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
  endif()
endfunction()

download(yolox-tiny.onnx 97028baf73ce55e115599c9c60651b08)
download(label.txt 9ceadca8b72b6169ee6aabb861fe3e1e)

##########
# tensorrt_yolox
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tensorrt_yolox.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
)

target_link_libraries(${PROJECT_NAME}
  ${tensorrt_common_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

ament_auto_add_library(yolox_single_image_inference_node SHARED
  src/yolox_single_image_inference_node.cpp
)

ament_target_dependencies(yolox_single_image_inference_node
  OpenCV
)

target_link_libraries(yolox_single_image_inference_node
  ${PROJECT_NAME}
  stdc++fs
)

target_compile_definitions(yolox_single_image_inference_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(yolox_single_image_inference_node
  PLUGIN "tensorrt_yolox::YoloXSingleImageInferenceNode"
  EXECUTABLE yolox_single_image_inference
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/tensorrt_yolox_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  OpenCV
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

target_compile_definitions(${PROJECT_NAME}_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "tensorrt_yolox::TrtYoloXNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  data
)
