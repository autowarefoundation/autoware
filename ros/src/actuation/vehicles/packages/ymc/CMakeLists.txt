cmake_minimum_required(VERSION 2.8.3)
project(ymc)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(
        catkin REQUIRED COMPONENTS
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


EXECUTE_PROCESS(
        COMMAND uname -m
        OUTPUT_VARIABLE ARCHITECTURE
        OUTPUT_STRIP_TRAILING_WHITESPACE
)


IF ("${ARCHITECTURE}" STREQUAL "aarch64")
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

find_library(ymc_can NAMES libymc_can_${LIB_VERSION}${LIB_ARCH}.a PATHS lib)
add_executable(g30esli_interface
        node/g30esli_interface/g30esli_interface.cpp
        )

target_link_libraries(g30esli_interface
        ${catkin_LIBRARIES}
        ${ymc_can}
        )
