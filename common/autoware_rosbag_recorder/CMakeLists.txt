cmake_minimum_required(VERSION 3.5)
project(autoware_rosbag_recorder)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

install(PROGRAMS
  scripts/record.sh
  DESTINATION lib/${PROJECT_NAME}
)
