cmake_minimum_required(VERSION 3.5)
project(log_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    autoware_build_flags
    )

catkin_package()
