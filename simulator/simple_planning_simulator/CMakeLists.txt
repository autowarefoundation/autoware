cmake_minimum_required(VERSION 3.6)
project(simple_planning_simulator)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Component
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/simple_planning_simulator/simple_planning_simulator_core.hpp
  include/simple_planning_simulator/visibility_control.hpp
  src/simple_planning_simulator/simple_planning_simulator_core.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_interface.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_vel.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast) # RCLCPP_ERROR_THROTTLE() has built-in old-style casts.

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

# Node executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "simulation::simple_planning_simulator::SimplePlanningSimulator"
  EXECUTABLE ${PROJECT_NAME}_exe
)

if(BUILD_TESTING)
  ament_add_gtest(test_simple_planning_simulator
    test/test_simple_planning_simulator.cpp
    TIMEOUT 120
  )

  target_link_libraries(test_simple_planning_simulator
    ${PROJECT_NAME}
  )

  # workaround to allow deprecated header to build on both galactic and rolling
  if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
    target_compile_definitions(test_simple_planning_simulator PRIVATE
      USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
    )
  endif()
endif()

ament_auto_package(INSTALL_TO_SHARE param launch)
