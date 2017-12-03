cmake_minimum_required(VERSION 2.8.3)
project(dbw_mkz_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(DIRECTORY msg FILES
  AmbientLight.msg
  BrakeCmd.msg
  BrakeInfoReport.msg
  BrakeReport.msg
  FuelLevelReport.msg
  Gear.msg
  GearCmd.msg
  GearReject.msg
  GearReport.msg
  HillStartAssist.msg
  Misc1Report.msg
  ParkingBrake.msg
  SteeringCmd.msg
  SteeringReport.msg
  SurroundReport.msg
  ThrottleCmd.msg
  ThrottleInfoReport.msg
  ThrottleReport.msg
  TirePressureReport.msg
  TurnSignal.msg
  TurnSignalCmd.msg
  TwistCmd.msg
  WatchdogCounter.msg
  WheelPositionReport.msg
  WheelSpeedReport.msg
  Wiper.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
