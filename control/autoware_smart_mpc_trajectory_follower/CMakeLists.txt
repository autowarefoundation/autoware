cmake_minimum_required(VERSION 3.14)
project(autoware_smart_mpc_trajectory_follower)

find_package(autoware_cmake REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
autoware_package()

execute_process(COMMAND bash -c "pip3 install numba==0.58.1 --force-reinstall")
execute_process(COMMAND bash -c "pip3 install GPy")

pybind11_add_module(proxima_calc SHARED
  ${PROJECT_NAME}/src/proxima_calc.cpp
)
ament_target_dependencies(proxima_calc Eigen3)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS proxima_calc
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/pympc_trajectory_follower.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_auto_package(
  INSTALL_TO_SHARE
  autoware_smart_mpc_trajectory_follower/param
)
