cmake_minimum_required(VERSION 2.8.12)
project(range_fusion)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  std_msgs
  autoware_msgs
  fusion
  )
FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  autoware_msgs
  fusion
  )

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
)

## range_fusion ##
add_executable(range_fusion
        nodes/range_fusion/range_fusion.cpp
        )

target_link_libraries(range_fusion
        ${catkin_LIBRARIES}
        )

add_dependencies(range_fusion
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS range_fusion
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
