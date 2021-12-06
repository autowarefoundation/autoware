# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(trajectory_follower)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# lateral_controller
set(LATERAL_CONTROLLER_LIB lateral_controller_lib)
set(LATERAL_CONTROLLER_LIB_SRC
  src/interpolate.cpp
  src/lowpass_filter.cpp
  src/mpc.cpp
  src/mpc_trajectory.cpp
  src/mpc_utils.cpp
  src/qp_solver/qp_solver_osqp.cpp
  src/qp_solver/qp_solver_unconstr_fast.cpp
  src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/vehicle_model/vehicle_model_interface.cpp
)

set(LATERAL_CONTROLLER_LIB_HEADERS
  include/trajectory_follower/visibility_control.hpp
  include/trajectory_follower/interpolate.hpp
  include/trajectory_follower/lowpass_filter.hpp
  include/trajectory_follower/mpc.hpp
  include/trajectory_follower/mpc_trajectory.hpp
  include/trajectory_follower/mpc_utils.hpp
  include/trajectory_follower/qp_solver/qp_solver_interface.hpp
  include/trajectory_follower/qp_solver/qp_solver_osqp.hpp
  include/trajectory_follower/qp_solver/qp_solver_unconstr_fast.hpp
  include/trajectory_follower/vehicle_model/vehicle_model_bicycle_dynamics.hpp
  include/trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp
  include/trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp
  include/trajectory_follower/vehicle_model/vehicle_model_interface.hpp
)

# generate library
ament_auto_add_library(${LATERAL_CONTROLLER_LIB} SHARED
  ${LATERAL_CONTROLLER_LIB_SRC}
  ${LATERAL_CONTROLLER_LIB_HEADERS}
)
autoware_set_compile_options(${LATERAL_CONTROLLER_LIB})
target_compile_options(${LATERAL_CONTROLLER_LIB} PRIVATE -Wno-error=old-style-cast)

# longitudinal_controller
set(LONGITUDINAL_CONTROLLER_LIB longitudinal_controller_lib)
set(LONGITUDINAL_CONTROLLER_LIB_SRC
  src/pid.cpp
  src/smooth_stop.cpp
  src/longitudinal_controller_utils.cpp
)

set(LONGITUDINAL_CONTROLLER_LIB_HEADERS
  include/trajectory_follower/debug_values.hpp
  include/trajectory_follower/pid.hpp
  include/trajectory_follower/smooth_stop.hpp
  include/trajectory_follower/longitudinal_controller_utils.hpp
)

# generate library
ament_auto_add_library(${LONGITUDINAL_CONTROLLER_LIB} SHARED
  ${LONGITUDINAL_CONTROLLER_LIB_SRC}
  ${LONGITUDINAL_CONTROLLER_LIB_HEADERS}
)
autoware_set_compile_options(${LONGITUDINAL_CONTROLLER_LIB})
target_compile_options(${LONGITUDINAL_CONTROLLER_LIB} PRIVATE -Wno-error=old-style-cast)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_LAT_SOURCES
    test/test_mpc.cpp
    test/test_mpc_trajectory.cpp
    test/test_mpc_utils.cpp
    test/test_interpolate.cpp
    test/test_lowpass_filter.cpp
  )
  set(TEST_LATERAL_CONTROLLER_EXE test_lateral_controller)
  ament_add_gtest(${TEST_LATERAL_CONTROLLER_EXE} ${TEST_LAT_SOURCES})
  autoware_set_compile_options(${TEST_LATERAL_CONTROLLER_EXE})
  target_link_libraries(${TEST_LATERAL_CONTROLLER_EXE} ${LATERAL_CONTROLLER_LIB})

  set(TEST_LON_SOURCES
    test/test_debug_values.cpp
    test/test_pid.cpp
    test/test_smooth_stop.cpp
    test/test_longitudinal_controller_utils.cpp
  )
  set(TEST_LONGITUDINAL_CONTROLLER_EXE test_longitudinal_controller)
  ament_add_gtest(${TEST_LONGITUDINAL_CONTROLLER_EXE} ${TEST_LON_SOURCES})
  autoware_set_compile_options(${TEST_LONGITUDINAL_CONTROLLER_EXE})
  target_link_libraries(${TEST_LONGITUDINAL_CONTROLLER_EXE} ${LONGITUDINAL_CONTROLLER_LIB})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
)
