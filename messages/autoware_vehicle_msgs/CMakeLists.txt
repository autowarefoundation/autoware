cmake_minimum_required(VERSION 3.5)
project(autoware_vehicle_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuationCommand.msg"
  "msg/ActuationCommandStamped.msg"
  "msg/ActuationStatus.msg"
  "msg/ActuationStatusStamped.msg"
  "msg/BatteryStatus.msg"
  "msg/Engage.msg"
  "msg/VehicleCommand.msg"
  "msg/Shift.msg"
  "msg/ShiftStamped.msg"
  "msg/ControlMode.msg"
  "msg/TurnSignal.msg"
  "msg/Steering.msg"
  "msg/SteeringWheelStatusStamped.msg"
  "msg/VehicleEmergencyStamped.msg"
  "msg/RawVehicleCommand.msg"
  "msg/RawControlCommand.msg"
  "srv/UpdateAccelBrakeMap.srv"
  DEPENDENCIES std_msgs geometry_msgs autoware_control_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
