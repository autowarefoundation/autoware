cmake_minimum_required(VERSION 3.5)
project(libvectormap)


find_package(autoware_build_flags REQUIRED)

find_package(vector_map REQUIRED)
find_package(vector_map_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${vector_map_INCLUDE_DIRS}
        ${vector_map_msgs_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)


add_library(libvectormap STATIC
        src/vector_map.cpp
        )
target_link_libraries(libvectormap
        ${EIGEN3_LIBRARIES}
        )
ament_target_dependencies(libvectormap
        vector_map_msgs
        )

install(TARGETS libvectormap
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        PATTERN ".svn" EXCLUDE
        )

ament_export_dependencies(ament_cmake)
ament_export_dependencies(vector_map)
ament_export_dependencies(vector_map_msgs)

ament_export_include_directories(include)
ament_export_include_directories(${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(libvectormap)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()