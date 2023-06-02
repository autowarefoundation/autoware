# Copyright 2021 The Autoware Foundation
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
# Developed by Robotec.ai

# Add a smoke test
# :param package_name: name of the package to smoke test
# :type package_name: string
# :param package_exec: package executable to run during smoke test
# :type executable_name: string
# :param PARAM_FILENAMES: yaml filenames containing test parameters
# :type PARAM_FILENAMES: string
# :param EXECUTABLE_ARGUMENTS: arguments passed to tested executable
# :type EXECUTABLE_ARGUMENTS: string

function(add_smoke_test package_name executable_name)
  cmake_parse_arguments(PARSE_ARGV 2 smoke_test "" "PARAM_FILENAMES;TEST_PARAM_FILENAMES;
EXECUTABLE_ARGUMENTS;TARGET_INFIX" "")

  set(ARGUMENTS "arg_package:=${package_name}" "arg_package_exe:=${executable_name}")

  set(TARGET_INFIX "")  # OPTIONAL infix to distinguish multiple smoke tests for one node
  if(smoke_test_TARGET_INFIX)
    set(TARGET_INFIX "_${smoke_test_TARGET_INFIX}")
  endif()

  if(smoke_test_PARAM_FILENAMES)
    list(APPEND ARGUMENTS "arg_param_filenames:=${smoke_test_PARAM_FILENAMES}")
  endif()

  if(smoke_test_TEST_PARAM_FILENAMES)
    list(APPEND ARGUMENTS "arg_test_param_filenames:=${smoke_test_TEST_PARAM_FILENAMES}")
  endif()

  if(smoke_test_EXECUTABLE_ARGUMENTS)
    list(APPEND ARGUMENTS "arg_executable_arguments:=${smoke_test_EXECUTABLE_ARGUMENTS}")
  endif()

  add_ros_test(
    ${autoware_testing_DIR}/../autoware_testing/smoke_test.py
    TARGET "${executable_name}${TARGET_INFIX}_smoke_test"
    ARGS "${ARGUMENTS}"
    TIMEOUT "30"
  )
endfunction()
