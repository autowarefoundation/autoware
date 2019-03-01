cmake_minimum_required(VERSION 2.8.11)
project(amathutils_lib) # autoware math utility

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp autoware_msgs)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS core filters kdtree)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               amathutils_lib
               CATKIN_DEPENDS
               roscpp
               autoware_msgs)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${autoware_msgs_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

add_library(amathutils_lib
            src/Amathutils.cpp
            src/vector3d.cpp
            src/vector2d.cpp
            src/kdtree_search.cpp
            src/polygon.cpp
            src/xorshift128.cpp)

target_link_libraries(amathutils_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(amathutils_lib ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.hpp")

install(TARGETS amathutils_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED rostest)
  catkin_add_gtest(polygon-test test/src/polygon_test.cpp)
  if(TARGET polygon-test)
    target_link_libraries(polygon-test amathutils_lib ${catkin_LIBRARIES})
  endif()
endif()
