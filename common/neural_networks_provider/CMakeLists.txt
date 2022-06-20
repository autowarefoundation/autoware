# Copyright 2021-2022 Arm Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.14)
project(neural_networks_provider)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(SUPPORTED_ARCH "aarch64" "x86_64")
set(USER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/user/${CMAKE_SYSTEM_PROCESSOR})

set(PREFIX_DIR ${CMAKE_BINARY_DIR}/download)
file(MAKE_DIRECTORY ${PREFIX_DIR}/src/networks)

if(CMAKE_SYSTEM_PROCESSOR IN_LIST SUPPORTED_ARCH)
  if(DOWNLOAD_ARTIFACTS)
    set(EXEC_DOWNLOAD TRUE)
  else()
    set(EXEC_DOWNLOAD FALSE)
    message(WARNING "Skipped download (enable by setting DOWNLOAD_ARTIFACTS)")
  endif()
else()
  set(EXEC_DOWNLOAD FALSE)
  message(WARNING "Skipped download (unsupported architecture)")
endif()

add_custom_target(artifact_download ALL
  COMMAND "EXEC_DOWNLOAD=${EXEC_DOWNLOAD}"
          "PREFIX_DIR=${PREFIX_DIR}"
          "CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}"
          "${CMAKE_CURRENT_SOURCE_DIR}/download_neural_networks.sh"
)

# Install the pre-compiled networks
install(
  DIRECTORY ${PREFIX_DIR}/src/networks
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# Install user-provided networks over the pre-compiled ones
if(EXISTS ${USER_DIR})
  install(
    DIRECTORY ${USER_DIR}/
    DESTINATION share/${PROJECT_NAME}/networks/
    USE_SOURCE_PERMISSIONS
  )
endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "neural_networks_provider-extras.cmake")
ament_auto_package()
