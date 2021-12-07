# vehicle_cmd_gate

## Purpose

`vehicle_cmd_gate` is the package to get information from emergency handler, planning module, external controller, and send a msg to vehicle.

## Inputs / Outputs

### Input

| Name                                        | Type                                                       | Description                                                          |
| ------------------------------------------- | ---------------------------------------------------------- | -------------------------------------------------------------------- |
| `~/input/steering`                          | `autoware_auto_vehicle_msgs::msg::SteeringReport`          | steering status                                                      |
| `~/input/auto/control_cmd`                  | `autoware_auto_control_msgs::msg::AckermannControlCommand` | command for lateral and longitudinal velocity from planning module   |
| `~/input/auto/turn_indicators_cmd`          | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand`   | turn indicators command from planning module                         |
| `~/input/auto/hazard_lights_cmd`            | `autoware_auto_vehicle_msgs::msg::HazardLightsCommand`     | hazard lights command from planning module                           |
| `~/input/auto/gear_cmd`                     | `autoware_auto_vehicle_msgs::msg::GearCommand`             | gear command from planning module                                    |
| `~/input/external/control_cmd`              | `autoware_auto_control_msgs::msg::AckermannControlCommand` | command for lateral and longitudinal velocity from external          |
| `~/input/external/turn_indicators_cmd`      | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand`   | turn indicators command from external                                |
| `~/input/external/hazard_lights_cmd`        | `autoware_auto_vehicle_msgs::msg::HazardLightsCommand`     | hazard lights command from external                                  |
| `~/input/external/gear_cmd`                 | `autoware_auto_vehicle_msgs::msg::GearCommand`             | gear command from external                                           |
| `~/input/external_emergency_stop_heartbeat` | `autoware_external_api_msgs::msg::Heartbeat`               | heartbeat                                                            |
| `~/input/gate_mode`                         | `autoware_control_msgs::msg::GateMode`                     | gate mode (AUTO or EXTERNAL)                                         |
| `~/input/emergency_state`                   | `autoware_auto_system_msgs::msg::EmergencyState`           | used to detect the emergency situation of the vehicle                |
| `~/input/emergency/control_cmd`             | `autoware_auto_control_msgs::msg::AckermannControlCommand` | command for lateral and longitudinal velocity from emergency handler |
| `~/input/emergency/hazard_lights_cmd`       | `autoware_auto_vehicle_msgs::msg::HazardLightsCommand`     | hazard lights command from emergency handler                         |
| `~/input/emergency/gear_cmd`                | `autoware_auto_vehicle_msgs::msg::GearCommand`             | gear command from emergency handler                                  |
| `~/input/engage`                            | `autoware_auto_vehicle_msgs::msg::Engage`                  | engage signal                                                        |

### Output

| Name                                   | Type                                                       | Description                                              |
| -------------------------------------- | ---------------------------------------------------------- | -------------------------------------------------------- |
| `~/output/vehicle_cmd_emergency`       | `autoware_auto_system_msgs::msg::EmergencyState`           | emergency state which was originally in vehicle command  |
| `~/output/command/control_cmd`         | `autoware_auto_control_msgs::msg::AckermannControlCommand` | command for lateral and longitudinal velocity to vehicle |
| `~/output/command/turn_indicators_cmd` | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand`   | turn indicators command to vehicle                       |
| `~/output/command/hazard_lights_cmd`   | `autoware_auto_vehicle_msgs::msg::HazardLightsCommand`     | hazard lights command to vehicle                         |
| `~/output/command/gear_cmd`            | `autoware_auto_vehicle_msgs::msg::GearCommand`             | gear command to vehicle                                  |
| `~/output/gate_mode`                   | `autoware_control_msgs::msg::GateMode`                     | gate mode (AUTO or EXTERNAL)                             |
| `~/output/engage`                      | `autoware_auto_vehicle_msgs::msg::Engage`                  | engage signal                                            |
| `~/output/external_emergency`          | `autoware_external_api_msgs::msg::Emergency`               | external emergency signal                                |

## Parameters

| Parameter                                   | Type   | Description                                                    |
| ------------------------------------------- | ------ | -------------------------------------------------------------- |
| `update_period`                             | double | update period                                                  |
| `use_emergency_handling`                    | bool   | true when emergency handler is used                            |
| `use_external_emergency_stop`               | bool   | true when external emergency stop information is used          |
| `system_emergency_heartbeat_timeout`        | double | timeout for system emergency                                   |
| `external_emergency_stop_heartbeat_timeout` | double | timeout for external emergency                                 |
| `stop_hold_acceleration`                    | double | longitudinal acceleration cmd when vehicle should stop         |
| `emergency_acceleration`                    | double | longitudinal acceleration cmd when vehicle stop with emergency |
| `vel_lim`                                   | double | limit of longitudinal velocity                                 |
| `lon_acc_lim`                               | double | limit of longitudinal acceleration                             |
| `lon_jerk_lim`                              | double | limit of longitudinal jerk                                     |
| `lat_acc_lim`                               | double | limit of lateral acceleration                                  |
| `lat_jerk_lim`                              | double | limit of lateral jerk                                          |

## Assumptions / Known limits

TBD.
