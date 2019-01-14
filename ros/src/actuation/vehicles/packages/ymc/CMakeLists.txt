cmake_minimum_required(VERSION 2.8.3)
project(ymc)

set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")

find_package(
        catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        geometry_msgs
        rosconsole
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        geometry_msgs
        rosconsole
)


IF ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    set(LIB_ARCH _aarch64)
ELSE ()
    unset(LIB_ARCH)
ENDIF ()

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

IF (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.1)
    set(LIB_VERSION 2.0) # _GLIBCXX_USE_CXX11_ABI is 1
ELSE ()
    set(LIB_VERSION 1.0) # _GLIBCXX_USE_CXX11_ABI is 0
ENDIF ()

add_executable(g30esli_interface
        node/g30esli_interface/g30esli_interface.cpp
        )

target_link_libraries(g30esli_interface
        ${catkin_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libymc_can_${LIB_VERSION}${LIB_ARCH}.a
        )
