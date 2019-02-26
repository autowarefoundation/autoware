cmake_minimum_required(VERSION 3.5)
project(diag_lib)

add_compile_options(-std=c++11)

find_package(diag_msgs REQUIRED)
find_package(roscpp REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

option(STRICT_ERROR_CODE_CHECK "enables strict error code check" OFF)
if(STRICT_ERROR_CODE_CHECK)
  add_definitions(-DSTRICT_ERROR_CODE_CHECK)
endif()

include_directories(
  include
  ${diag_msgs_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

set(watchdog_node_src src/watchdog_node.cpp src/watchdog.cpp src/diag_manager.cpp src/rate_checker.cpp src/diag_subscriber.cpp)
add_executable(watchdog_node ${watchdog_node_src})
target_link_libraries(watchdog_node ${YAML_CPP_LIBRARIES})
ament_target_dependencies(watchdog_node diag_msgs roscpp)

set(diag_manager_src src/diag_manager.cpp src/rate_checker.cpp src/diag_subscriber.cpp)
add_library(diag_manager SHARED ${diag_manager_src})
target_link_libraries(diag_manager ${YAML_CPP_LIBRARIES})
ament_target_dependencies(diag_manager diag_msgs roscpp)

set(diag_filter_src src/diag_filter.cpp)
add_library(diag_filter SHARED ${diag_filter_src})
target_link_libraries(diag_filter ${YAML_CPP_LIBRARIES})
ament_target_dependencies(diag_filter diag_msgs roscpp)

# CPP Execution programs
set(CPP_EXEC_NAMES watchdog_node)
foreach(cpp_exec_names ${CPP_EXEC_NAMES})
  install(TARGETS ${cpp_exec_names}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
endforeach(cpp_exec_names)

# include header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install all files
foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

# Install library
install(TARGETS diag_manager diag_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(diag_msgs)
ament_export_dependencies(roscpp)

ament_export_include_directories(include)

ament_export_libraries(diag_manager)
ament_export_libraries(diag_filter)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
