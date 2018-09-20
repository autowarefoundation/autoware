cmake_minimum_required(VERSION 2.8.3)
project(diag_lib)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  diag_msgs
  roscpp
)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

option(STRICT_ERROR_CODE_CHECK "enables strict error code check" OFF)
if(STRICT_ERROR_CODE_CHECK)
  add_definitions(-DSTRICT_ERROR_CODE_CHECK)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diag_manager diag_filter
  CATKIN_DEPENDS diag_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(watchdog_node_src src/watchdog_node.cpp src/watchdog.cpp src/diag_manager.cpp src/rate_checker.cpp src/diag_subscriber.cpp)
add_executable(watchdog_node ${watchdog_node_src})
target_link_libraries(watchdog_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(watchdog_node ${catkin_EXPORTED_TARGETS})

set(diag_manager_src src/diag_manager.cpp src/rate_checker.cpp src/diag_subscriber.cpp)
add_library(diag_manager SHARED ${diag_manager_src})
target_link_libraries(diag_manager ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(diag_manager ${catkin_EXPORTED_TARGETS})

set(diag_filter_src src/diag_filter.cpp)
add_library(diag_filter SHARED ${diag_filter_src})
target_link_libraries(diag_filter ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(diag_filter ${catkin_EXPORTED_TARGETS})

# CPP Execution programs
set(CPP_EXEC_NAMES watchdog_node)
foreach(cpp_exec_names ${CPP_EXEC_NAMES})
  install(TARGETS ${cpp_exec_names}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach(cpp_exec_names)

# include header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install all files
foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Install library
install(TARGETS diag_manager diag_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)