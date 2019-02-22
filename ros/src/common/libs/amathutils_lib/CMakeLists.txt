cmake_minimum_required(VERSION 3.5)
project(amathutils_lib) # autoware math utility

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(roscpp REQUIRED)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(amathutils_lib
        include
        ${autoware_msgs_INCLUDE_DIRS}
        )

add_library(amathutils_lib
        src/Amathutils.cpp
        )

ament_target_dependencies(amathutils_lib
        autoware_msgs
        roscpp
        )

install(
        DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp"
        )

install(TARGETS amathutils_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

ament_export_dependencies(ament_cmake)
ament_export_dependencies(autoware_msgs)
ament_export_dependencies(roscpp)

ament_export_include_directories(include)

ament_export_libraries(amathutils_lib)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()