cmake_minimum_required(VERSION 3.5)
project(roboteq_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # Autoware関連やEigenを使う場合は17を推奨
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # -Werrorはデバッグ中一旦外すとスムーズです
endif()

find_package(ament_cmake_auto REQUIRED)

# Eigen3と追加したAutowareのパッケージを明示的に探す
find_package(Eigen3 REQUIRED)
find_package(autoware_vehicle_info_utils REQUIRED)

# 依存関係を自動スキャン（package.xmlの記述に基づきます）
ament_auto_find_build_dependencies()

# ライブラリのビルド
ament_auto_add_library(roboteq_interface_node SHARED
  src/roboteq_interface.cpp
  src/roboteq_interface_diagnostic.cpp
)

# Eigenのインクルードパスを明示的に追加（ament_autoが漏らす場合があるため）
target_include_directories(roboteq_interface_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

rclcpp_components_register_node(roboteq_interface_node
  PLUGIN "RoboteqInterface"
  EXECUTABLE roboteq_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)