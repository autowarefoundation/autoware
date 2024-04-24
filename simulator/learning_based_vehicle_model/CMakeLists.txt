cmake_minimum_required(VERSION 3.14.4)
project(learning_based_vehicle_model)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_link_libraries(${PROJECT_NAME} pybind11::embed ${Python3_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS})

target_compile_options(${PROJECT_NAME} PRIVATE -fvisibility=hidden)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_auto_package()
