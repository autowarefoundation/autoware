cmake_minimum_required(VERSION 3.5)
project(localization_evaluator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/metrics_calculator.cpp
  src/${PROJECT_NAME}_node.cpp
  src/localization_evaluator_node.cpp
  src/metrics/localization_metrics.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "localization_diagnostics::LocalizationEvaluatorNode"
  EXECUTABLE ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_localization_evaluator_node.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
