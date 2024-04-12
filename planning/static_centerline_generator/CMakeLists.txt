cmake_minimum_required(VERSION 3.14)
project(static_centerline_generator)

find_package(autoware_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  static_centerline_generator
  "srv/LoadMap.srv"
  "srv/PlanRoute.srv"
  "srv/PlanPath.srv"
  "msg/PointsWithLaneId.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

autoware_package()

ament_auto_add_executable(main
  src/main.cpp
  src/static_centerline_generator_node.cpp
  src/centerline_source/optimization_trajectory_based_centerline.cpp
  src/centerline_source/bag_ego_trajectory_based_centerline.cpp
  src/utils.cpp
)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(main
    static_centerline_generator "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target static_centerline_generator "rosidl_typesupport_cpp")
    target_link_libraries(main "${cpp_typesupport_target}")
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    rviz
)

if(BUILD_TESTING)
  add_launch_test(
    test/test_static_centerline_generator.test.py
    TIMEOUT "30"
  )
  install(DIRECTORY
    test/data/
    DESTINATION share/${PROJECT_NAME}/test/data/
  )
endif()

install(PROGRAMS
  scripts/app.py
  DESTINATION lib/${PROJECT_NAME}
)
