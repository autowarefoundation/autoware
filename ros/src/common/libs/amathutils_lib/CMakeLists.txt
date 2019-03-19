cmake_minimum_required(VERSION 2.8.11)
project(amathutils_lib) # autoware math utility

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        autoware_msgs
        )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES amathutils_lib
        CATKIN_DEPENDS roscpp autoware_msgs
)

include_directories(amathutils_lib
        include
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        )

add_library(amathutils_lib
        src/Amathutils.cpp
        )
target_link_libraries(amathutils_lib
        ${catkin_LIBRARIES}
        )

add_dependencies(amathutils_lib
        ${catkin_EXPORTED_TARGETS}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )

install(TARGETS amathutils_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(amathutils-test test/test_amathutils_lib.test test/src/test_amathutils_lib.cpp)
  target_link_libraries(amathutils-test ${catkin_LIBRARIES} amathutils_lib)
endif()
