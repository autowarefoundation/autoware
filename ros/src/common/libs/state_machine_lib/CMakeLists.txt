cmake_minimum_required(VERSION 3.5)
project(state_machine_lib)

find_package(autoware_build_flags REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(roscpp REQUIRED)
find_package(ament_cmake REQUIRED)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${autoware_msgs_INCLUDE_DIRS}
)

add_library(state_machine_lib
        src/state_context.cpp
        )
ament_target_dependencies(state_machine_lib
        autoware_msgs
        roscpp
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.hpp"
        )


install(TARGETS state_machine_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

ament_export_dependencies(ament_cmake)
ament_export_dependencies(autoware_msgs)
ament_export_dependencies(roscpp)

ament_export_include_directories(include)

ament_export_libraries(state_machine_lib)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()