cmake_minimum_required(VERSION 2.8.12)

project(costmap_generator)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  pcl_ros
  grid_map_ros
  tf
  autoware_msgs
  vector_map
  object_map
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pcl_ros
    grid_map_ros
    tf
    autoware_msgs
    vector_map
    object_map
)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

### costmap_generator ###
add_library(costmap_generator_lib
        nodes/costmap_generator/costmap_generator_node.cpp
        nodes/costmap_generator/costmap_generator.cpp
        nodes/costmap_generator/points_to_costmap.cpp
        nodes/costmap_generator/objects_to_costmap.cpp
        )
target_link_libraries(costmap_generator_lib
        ${catkin_LIBRARIES}
        )
add_dependencies(costmap_generator_lib
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(costmap_generator
        nodes/costmap_generator/costmap_generator_node.cpp
        nodes/costmap_generator/costmap_generator.cpp
        nodes/costmap_generator/points_to_costmap.cpp
        nodes/costmap_generator/objects_to_costmap.cpp
        )

target_link_libraries(costmap_generator
        ${catkin_LIBRARIES}
        )

add_dependencies(costmap_generator
        ${catkin_EXPORTED_TARGETS}
        )


install(TARGETS costmap_generator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test-costmap_generator test/src/test_costmap_generator.cpp)
  target_link_libraries(test-costmap_generator ${catkin_LIBRARIES} costmap_generator_lib)
endif()
