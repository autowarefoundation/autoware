cmake_minimum_required(VERSION 2.8.3)
project(ymc)

find_package(
  catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  rosconsole
  geometry_msgs
  autoware_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rosconsole
  geometry_msgs
  autoware_msgs
)

IF ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
  set(LIB_ARCH _aarch64)
ELSE ()
  unset(LIB_ARCH)
ENDIF ()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(g30esli_interface
  node/g30esli_interface/g30esli_interface.cpp
)

add_dependencies(g30esli_interface
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(g30esli_interface
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libymc_can${LIB_ARCH}.a
)

install(TARGETS g30esli_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
