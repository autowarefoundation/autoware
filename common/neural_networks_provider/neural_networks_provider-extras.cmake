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

normalize_path(neural_networks_provider_NETWORKS_DIR
  "${neural_networks_provider_DIR}/../networks"
)
if(NOT IS_DIRECTORY "${neural_networks_provider_NETWORKS_DIR}")
  message(WARNING "Package 'neural_networks_provider' exports the directory '${neural_networks_provider_NETWORKS_DIR}' which doesn't exist")
endif()

macro(_subdirlist result dir)
  file(GLOB children RELATIVE ${dir} ${dir}/*)
  set(${result} "")
  foreach(child ${children})
    if(IS_DIRECTORY ${dir}/${child})
      list(APPEND ${result} ${child})
    endif()
  endforeach()
endmacro()

_subdirlist(neural_networks_provider_NAMES ${neural_networks_provider_NETWORKS_DIR})
