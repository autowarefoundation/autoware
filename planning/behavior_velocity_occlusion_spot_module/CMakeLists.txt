cmake_minimum_required(VERSION 3.14)
project(behavior_velocity_occlusion_spot_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(behavior_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/grid_utils.cpp
  src/manager.cpp
  src/occlusion_spot_utils.cpp
  src/risk_predictive_braking.cpp
  src/scene_occlusion_spot.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/src/test_occlusion_spot_utils.cpp
    test/src/test_risk_predictive_braking.cpp
    test/src/test_grid_utils.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE config)
