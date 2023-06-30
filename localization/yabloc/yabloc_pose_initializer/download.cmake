# Copyright 2023 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(DOWNLOAD_ARTIFACTS OFF CACHE BOOL "enable artifacts download")

set(DATA_URL "https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz")
set(DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
set(FILE_HASH 146ed8af689a30b898dc5369870c40fb)
set(FILE_NAME "resources.tar.gz")

function(download_and_extract)
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
      file(DOWNLOAD ${DATA_URL} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 3600)
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    endif()
  else()
    if(DOWNLOAD_ARTIFACTS)
      message(STATUS "not found ${FILE_NAME}")
      message(STATUS "File doesn't exists. Downloading now ...")
      file(DOWNLOAD ${DATA_URL} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 3600)
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    else()
      message(WARNING "Skipped download for ${FILE_NAME} (enable by setting DOWNLOAD_ARTIFACTS)")
      file(MAKE_DIRECTORY "${DATA_PATH}")
      return()
    endif()
  endif()

  if(${STATUS_CODE} EQUAL 0)
    message(STATUS "Download completed successfully!")
  else()
    message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
  endif()

  execute_process(COMMAND
    ${CMAKE_COMMAND} -E
    tar xzf "${DATA_PATH}/${FILE_NAME}" WORKING_DIRECTORY "${DATA_PATH}")
endfunction()

download_and_extract()
