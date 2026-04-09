# ROBOTEQ_INTERFACE

## Input / Output

### Input topics

- From Autoware

| Name                                 | Type                                                   | Description                              |
| :----------------------------------- | :----------------------------------------------------- | :--------------------------------------- |
| /control/command/control_cmd         | autoware_auto_control_msgs/msg/AckermannControlCommand | lateral and longitudinal control command |
| /control/command/gear_cmd            | autoware_auto_vehicle_msgs/msg/GearCommand             | gear command                             |
| /control/command/turn_indicators_cmd | autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand   | turn indicators command                  |
| /control/command/hazard_lights_cmd   | autoware_auto_vehicle_msgs/msg/HazardLightsCommand     | hazard lights command                    |
| /control/command/emergency_cmd       | tier4_vehicle_msgs/msg/VehicleEmergencyStamped         | emergency command                        |

- From roboteq_serial_interface

| Name                  | Type                                  | Description                                |
| :-------------------- | :------------------------------------ | :----------------------------------------- |
| /roboteq/left/status  | roboteq_msgs/msg/RoboteqStatusStamped | current status from left motor controller  |
| /roboteq/right/status | roboteq_msgs/msg/RoboteqStatusStamped | current status from right motor controller |

---

### Output topics

- To Autoware

| Name                                   | Type                                                | Description                |
| :------------------------------------- | :-------------------------------------------------- | :------------------------- |
| /vehicle/status/velocity_status        | autoware_auto_vehicle_msgs/msg/VelocityReport       | velocity                   |
| /vehicle/status/steering_status        | autoware_auto_vehicle_msgs/msg/SteeringReport       | steering wheel angle       |
| /vehicle/status/gear_status            | autoware_auto_vehicle_msgs/msg/GearReport           | gear status                |
| /vehicle/status/turn_indicators_status | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport | turn indicators status     |
| /vehicle/status/hazard_lights_status   | autoware_auto_vehicle_msgs/msg/HazardLightsReport   | hazard lights status       |
| /vehicle/status/control_mode           | autoware_auto_vehicle_msgs/msg/ControlModeReport    | control mode               |
| /vehicle/status/velocity_kmph          | tier4_debug_msgs/msg/Float32Stamped                 | velocity [km/h]            |
| /vehicle/status/steering_wheel_deg     | tier4_debug_msgs/msg/Float32Stamped                 | steering wheel angle [deg] |
| /vehicle/status/battery_charge         | tier4_vehicle_msgs/msg/BatteryStatus                | **NOT IMPLEMENTED**        |

- To roboteq_serial_interface

| Name                   | Type                                   | Description                       |
| :--------------------- | :------------------------------------- | :-------------------------------- |
| /roboteq/left/command  | roboteq_msgs/msg/RoboteqCommandStamped | command to left motor controller  |
| /roboteq/right/command | roboteq_msgs/msg/RoboteqCommandStamped | command to right motor controller |

## ROS Parameters

| Name                       | Type   | Description                        | Default |
| :------------------------- | :----- | :--------------------------------- | :------ |
| loop_rate                  | double | loop rate to publish commands [Hz] | 50.0    |
| control_cmd_timeout_sec    | double | control_cmd time out [sec]         | 1.0     |
| roboteq_status_timeout_sec | double | roboteq_status time out [sec]      | 0.5     |
