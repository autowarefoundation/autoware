cmake_minimum_required(VERSION 3.14)
project(trajectory_follower)

find_package(autoware_cmake REQUIRED)
autoware_package()

# lateral_controller
set(LATERAL_CONTROLLER_LIB lateral_controller_lib)
set(LATERAL_CONTROLLER_LIB_SRC
  src/mpc_lateral_controller.cpp
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
  include/trajectory_follower/lateral_controller_base.hpp
  include/trajectory_follower/mpc_lateral_controller.hpp
  include/trajectory_follower/sync_data.hpp
  include/trajectory_follower/input_data.hpp
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

ament_auto_add_library(${LATERAL_CONTROLLER_LIB} SHARED
  ${LATERAL_CONTROLLER_LIB_SRC}
  ${LATERAL_CONTROLLER_LIB_HEADERS}
)
target_compile_options(${LATERAL_CONTROLLER_LIB} PRIVATE -Wno-error=old-style-cast)

# longitudinal_controller
set(LONGITUDINAL_CONTROLLER_LIB longitudinal_controller_lib)
set(LONGITUDINAL_CONTROLLER_LIB_SRC
  src/pid_longitudinal_controller.cpp
  src/pid.cpp
  src/smooth_stop.cpp
  src/longitudinal_controller_utils.cpp
)

set(LONGITUDINAL_CONTROLLER_LIB_HEADERS
  include/trajectory_follower/longitudinal_controller_base.hpp
  include/trajectory_follower/pid_longitudinal_controller.hpp
  include/trajectory_follower/sync_data.hpp
  include/trajectory_follower/input_data.hpp
  include/trajectory_follower/debug_values.hpp
  include/trajectory_follower/pid.hpp
  include/trajectory_follower/smooth_stop.hpp
  include/trajectory_follower/longitudinal_controller_utils.hpp
)

ament_auto_add_library(${LONGITUDINAL_CONTROLLER_LIB} SHARED
  ${LONGITUDINAL_CONTROLLER_LIB_SRC}
  ${LONGITUDINAL_CONTROLLER_LIB_HEADERS}
)
target_compile_options(${LONGITUDINAL_CONTROLLER_LIB} PRIVATE -Wno-error=old-style-cast)

if(BUILD_TESTING)
  set(TEST_LAT_SOURCES
    test/test_mpc.cpp
    test/test_mpc_utils.cpp
    test/test_interpolate.cpp
    test/test_lowpass_filter.cpp
  )
  set(TEST_LATERAL_CONTROLLER_EXE test_lateral_controller)
  ament_add_ros_isolated_gtest(${TEST_LATERAL_CONTROLLER_EXE} ${TEST_LAT_SOURCES})
  target_link_libraries(${TEST_LATERAL_CONTROLLER_EXE} ${LATERAL_CONTROLLER_LIB})

  set(TEST_LON_SOURCES
    test/test_pid.cpp
    test/test_smooth_stop.cpp
    test/test_longitudinal_controller_utils.cpp
  )
  set(TEST_LONGITUDINAL_CONTROLLER_EXE test_longitudinal_controller)
  ament_add_ros_isolated_gtest(${TEST_LONGITUDINAL_CONTROLLER_EXE} ${TEST_LON_SOURCES})
  target_link_libraries(${TEST_LONGITUDINAL_CONTROLLER_EXE} ${LONGITUDINAL_CONTROLLER_LIB})
endif()

ament_auto_package(INSTALL_TO_SHARE
)
