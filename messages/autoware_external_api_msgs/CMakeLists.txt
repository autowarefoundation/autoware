cmake_minimum_required(VERSION 3.5)
project(autoware_external_api_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ClassifiedDiagnostics.msg
  msg/ControlCommand.msg
  msg/ControlCommandStamped.msg
  msg/DoorStatus.msg
  msg/Emergency.msg
  msg/EngageStatus.msg
  msg/FailSafeState.msg
  msg/FailSafeStateStamped.msg
  msg/GearShift.msg
  msg/GearShiftStamped.msg
  msg/Heartbeat.msg
  msg/MapHash.msg
  msg/MetadataPackages.msg
  msg/Observer.msg
  msg/Operator.msg
  msg/ResponseStatus.msg
  msg/Route.msg
  msg/RouteSection.msg
  msg/Service.msg
  msg/TextFile.msg
  msg/Steering.msg
  msg/TurnSignal.msg
  msg/TurnSignalStamped.msg
  msg/VehicleCommand.msg
  msg/VehicleCommandStamped.msg
  msg/VehicleStatus.msg
  msg/VehicleStatusStamped.msg
  srv/ClearRoute.srv
  srv/Engage.srv
  srv/GetMetadataPackages.srv
  srv/GetTextFile.srv
  srv/GetVersion.srv
  srv/InitializePose.srv
  srv/InitializePoseAuto.srv
  srv/PauseDriving.srv
  srv/SetDoor.srv
  srv/SetObserver.srv
  srv/SetOperator.srv
  srv/SetEmergency.srv
  srv/SetPose.srv
  srv/SetRoute.srv
  srv/SetService.srv
  srv/SetVelocityLimit.srv
  DEPENDENCIES
    builtin_interfaces
    diagnostic_msgs
    geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
