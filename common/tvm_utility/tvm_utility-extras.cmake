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

#
# Make inference_engine_tvm_config.hpp available under "data/models/${MODEL_NAME}/".
# Install the TVM artifacts to "share/${PROJECT_NAME}/models/".
# Return the name of the custom target in the DEPENDENCY parameter.
#
# :param MODEL_NAME: the name of the targeted neural network
# :type MODEL_NAME: string
# :param MODEL_BACKEND: the name of the targeted backend
# :type MODEL_BACKEND: string
# :param DEPENDENCY: output parameter; name of the ExternalProject top level target
# :type DEPENDENCY: string
#
function(get_neural_network MODEL_NAME MODEL_BACKEND DEPENDENCY)
  set(DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data)
  set(EXTERNALPROJECT_NAME ${MODEL_NAME}_${MODEL_BACKEND})
  set(PREPROCESSING "")

  if(IS_DIRECTORY "${DATA_PATH}/models/${MODEL_NAME}")
    set(SOURCE_DIR "${DATA_PATH}/models/${MODEL_NAME}")
    set(INSTALL_DIRECTORY "${DATA_PATH}/models/${MODEL_NAME}")
    if(EXISTS "${DATA_PATH}/models/${MODEL_NAME}/preprocessing_inference_engine_tvm_config.hpp")
      set(PREPROCESSING "${DATA_PATH}/models/${MODEL_NAME}/preprocessing_inference_engine_tvm_config.hpp")
    endif()
  else()
    message(WARNING "No model configuration files were provided")
    set(${DEPENDENCY} "" PARENT_SCOPE)
    return()
  endif()

  include(ExternalProject)
  externalproject_add(${EXTERNALPROJECT_NAME}
    SOURCE_DIR ${SOURCE_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    BUILD_BYPRODUCTS "${DATA_PATH}/models/${MODEL_NAME}/inference_engine_tvm_config.hpp"
    BUILD_BYPRODUCTS ${PREPROCESSING}
    INSTALL_COMMAND ""
  )
  install(
    DIRECTORY ${INSTALL_DIRECTORY}
    DESTINATION "share/${PROJECT_NAME}/models/"
    USE_SOURCE_PERMISSIONS
  )

  set(${DEPENDENCY} ${EXTERNALPROJECT_NAME} PARENT_SCOPE)

endfunction()
