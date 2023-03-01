cmake_minimum_required(VERSION 3.14)
project(static_centerline_optimizer)

find_package(autoware_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(
  static_centerline_optimizer
  "srv/LoadMap.srv"
  "srv/PlanRoute.srv"
  "srv/PlanPath.srv"
  "msg/PointsWithLaneId.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

autoware_package()

ament_auto_add_executable(main
  src/main.cpp
  src/static_centerline_optimizer_node.cpp
  src/successive_trajectory_optimizer_node.cpp
  src/utils.cpp
)

target_include_directories(main
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(main
  ${OpenCV_LIBRARIES}
)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(main
    static_centerline_optimizer "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target static_centerline_optimizer "rosidl_typesupport_cpp")
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
    test/test_static_centerline_optimizer.test.py
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
