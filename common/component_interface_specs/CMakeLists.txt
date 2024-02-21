cmake_minimum_required(VERSION 3.14)
project(component_interface_specs)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  ament_auto_add_gtest(gtest_${PROJECT_NAME}
    test/gtest_main.cpp
    test/test_planning.cpp
    test/test_control.cpp
    test/test_localization.cpp
    test/test_system.cpp
    test/test_map.cpp
    test/test_perception.cpp
    test/test_vehicle.cpp
  )
endif()

ament_auto_package()
