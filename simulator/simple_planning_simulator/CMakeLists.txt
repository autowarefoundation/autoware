cmake_minimum_required(VERSION 3.6)

project(simple_planning_simulator)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Component
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/simple_planning_simulator/simple_planning_simulator_core.hpp
  include/simple_planning_simulator/visibility_control.hpp
  src/simple_planning_simulator/simple_planning_simulator_core.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_interface.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_vel.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
autoware_set_compile_options(${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast) # RCLCPP_ERROR_THROTTLE() has built-in old-style casts.


# Node executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "simulation::simple_planning_simulator::SimplePlanningSimulator"
  EXECUTABLE ${PROJECT_NAME}_exe
)


### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()




ament_auto_package(INSTALL_TO_SHARE param launch)
