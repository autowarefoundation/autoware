# pacmod_interface

`pacmod_interface` is the package to connect Autoware with Pacmod.

## Input / Output

### Input topics

- From Autoware

  | Name                                   | Type                                                     | Description                                           |
  | -------------------------------------- | -------------------------------------------------------- | ----------------------------------------------------- |
  | `/control/command/control_cmd`         | autoware_auto_control_msgs::msg::AckermannControlCommand | lateral and longitudinal control command              |
  | `/control/command/gear_cmd`            | autoware_auto_vehicle_msgs::msg::GearCommand             | gear command                                          |
  | `/control/command/turn_indicators_cmd` | autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand   | turn indicators command                               |
  | `/control/command/hazard_lights_cmd`   | autoware_auto_vehicle_msgs::msg::HazardLightsCommand     | hazard lights command                                 |
  | `/vehicle/engage`                      | autoware_auto_vehicle_msgs::msg::Engage                  | engage command                                        |
  | `/vehicle/command/actuation_cmd`       | tier4_vehicle_msgs::msg::ActuationCommandStamped         | actuation (accel/brake pedal, steering wheel) command |
  | `/control/command/emergency_cmd`       | tier4_vehicle_msgs::msg::VehicleEmergencyStamped         | emergency command                                     |

- From Pacmod

  | Name                      | Type                              | Description                                                             |
  | ------------------------- | --------------------------------- | ----------------------------------------------------------------------- |
  | `/pacmod/steering_rpt`    | pacmod3_msgs::msg::SystemRptFloat | current steering wheel angle                                            |
  | `/pacmod/wheel_speed_rpt` | pacmod3_msgs::msg::WheelSpeedRpt  | current wheel speed                                                     |
  | `/pacmod/accel_rpt`       | pacmod3_msgs::msg::SystemRptFloat | current accel pedal                                                     |
  | `/pacmod/brake_rpt`       | pacmod3_msgs::msg::SystemRptFloat | current brake pedal                                                     |
  | `/pacmod/shift_rpt`       | pacmod3_msgs::msg::SystemRptInt   | current gear status                                                     |
  | `/pacmod/turn_rpt`        | pacmod3_msgs::msg::SystemRptInt   | current turn indicators status                                          |
  | `/pacmod/global_rpt`      | pacmod3_msgs::msg::GlobalRpt      | current status of other parameters (e.g. override_active, can_time_out) |

### Output topics

- To Pacmod

  | Name                   | Type                              | Description                                           |
  | ---------------------- | --------------------------------- | ----------------------------------------------------- |
  | `pacmod/accel_cmd`     | pacmod3_msgs::msg::SystemCmdFloat | accel pedal command                                   |
  | `pacmod/brake_cmd`     | pacmod3_msgs::msg::SystemCmdFloat | brake pedal command                                   |
  | `pacmod/steering_cmd`  | pacmod3_msgs::msg::SystemCmdFloat | steering wheel angle and angular velocity command     |
  | `pacmod/shift_cmd`     | pacmod3_msgs::msg::SystemCmdInt   | gear command                                          |
  | `pacmod/turn_cmd`      | pacmod3_msgs::msg::SystemCmdInt   | turn indicators command                               |
  | `pacmod/raw_steer_cmd` | pacmod3_msgs::msg::SteerSystemCmd | raw steering wheel angle and angular velocity command |

- To Autoware

  | Name                                     | Type                                                    | Description                                          |
  | ---------------------------------------- | ------------------------------------------------------- | ---------------------------------------------------- |
  | `/vehicle/status/control_mode`           | autoware_auto_vehicle_msgs::msg::ControlModeReport      | control mode                                         |
  | `/vehicle/status/velocity_status`        | autoware_auto_vehicle_msgs::msg::VelocityReport         | velocity                                             |
  | `/vehicle/status/steering_status`        | autoware_auto_vehicle_msgs::msg::SteeringReport         | steering wheel angle                                 |
  | `/vehicle/status/gear_status`            | autoware_auto_vehicle_msgs::msg::GearReport             | gear status                                          |
  | `/vehicle/status/turn_indicators_status` | autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport   | turn indicators status                               |
  | `/vehicle/status/hazard_lights_status`   | autoware_auto_vehicle_msgs::msg::HazardLightsReport     | hazard lights status                                 |
  | `/vehicle/status/actuation_status`       | autoware_auto_vehicle_msgs::msg::ActuationStatusStamped | actuation (accel/brake pedal, steering wheel) status |

## ROS Parameters

| Name                              | Type   | Description                                                                               |
| --------------------------------- | ------ | ----------------------------------------------------------------------------------------- |
| `base_frame_id`                   | string | frame id (assigned in pacmod command, but it does not make sense)                         |
| `command_timeout_ms`              | double | timeout [ms]                                                                              |
| `loop_rate`                       | double | loop rate to publish commands                                                             |
| `steering_offset`                 | double | steering wheel angle offset                                                               |
| `enable_steering_rate_control`    | bool   | when enabled, max steering wheel rate is used for steering wheel angular velocity command |
| `emergency_brake`                 | double | brake pedal for emergency                                                                 |
| `vgr_coef_a`                      | double | coefficient to calculate steering wheel angle                                             |
| `vgr_coef_b`                      | double | coefficient to calculate steering wheel angle                                             |
| `vgr_coef_c`                      | double | coefficient to calculate steering wheel angle                                             |
| `accel_pedal_offset`              | double | accel pedal offset                                                                        |
| `brake_pedal_offset`              | double | brake pedal offset                                                                        |
| `max_throttle`                    | double | max accel pedal                                                                           |
| `max_brake`                       | double | max brake pedal                                                                           |
| `max_steering_wheel`              | double | max steering wheel angle                                                                  |
| `max_steering_wheel_rate`         | double | max steering wheel angular velocity                                                       |
| `min_steering_wheel_rate`         | double | min steering wheel angular velocity                                                       |
| `steering_wheel_rate_low_vel`     | double | min steering wheel angular velocity when velocity is low                                  |
| `steering_wheel_rate_low_stopped` | double | min steering wheel angular velocity when velocity is almost 0                             |
| `low_vel_thresh`                  | double | threshold velocity to decide the velocity is low for `steering_wheel_rate_low_vel`        |
| `hazard_thresh_time`              | double | threshold time to keep hazard lights                                                      |
