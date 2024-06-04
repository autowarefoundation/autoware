# mrm_handler

## Purpose

MRM Handler is a node to select a proper MRM from a system failure state contained in OperationModeAvailability.

## Inner-workings / Algorithms

### State Transitions

![mrm-state](image/mrm-state.svg)

## Inputs / Outputs

### Input

| Name                                   | Type                                                | Description                                                                                         |
| -------------------------------------- | --------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| `/localization/kinematic_state`        | `nav_msgs::msg::Odometry`                           | Used to decide whether vehicle is stopped or not                                                    |
| `/system/operation_mode/availability`  | `tier4_system_msgs::msg::OperationModeAvailability` | Used to select proper MRM from system available mrm behavior contained in operationModeAvailability |
| `/vehicle/status/control_mode`         | `autoware_vehicle_msgs::msg::ControlModeReport`     | Used to check vehicle mode: autonomous or manual                                                    |
| `/system/mrm/emergency_stop/status`    | `tier4_system_msgs::msg::MrmBehaviorStatus`         | Used to check if MRM emergency stop operation is available                                          |
| `/system/mrm/comfortable_stop/status`  | `tier4_system_msgs::msg::MrmBehaviorStatus`         | Used to check if MRM comfortable stop operation is available                                        |
| `/system/mrm/pull_over_manager/status` | `tier4_system_msgs::msg::MrmBehaviorStatus`         | Used to check if MRM pull over operation is available                                               |
| `/api/operation_mode/state`            | `autoware_adapi_v1_msgs::msg::OperationModeState`   | Used to check whether the current operation mode is AUTO or STOP.                                   |

### Output

| Name                                    | Type                                              | Description                                           |
| --------------------------------------- | ------------------------------------------------- | ----------------------------------------------------- |
| `/system/emergency/gear_cmd`            | `autoware_vehicle_msgs::msg::GearCommand`         | Required to execute proper MRM (send gear cmd)        |
| `/system/emergency/hazard_lights_cmd`   | `autoware_vehicle_msgs::msg::HazardLightsCommand` | Required to execute proper MRM (send turn signal cmd) |
| `/system/fail_safe/mrm_state`           | `autoware_adapi_v1_msgs::msg::MrmState`           | Inform MRM execution state and selected MRM behavior  |
| `/system/mrm/emergency_stop/operate`    | `tier4_system_msgs::srv::OperateMrm`              | Execution order for MRM emergency stop                |
| `/system/mrm/comfortable_stop/operate`  | `tier4_system_msgs::srv::OperateMrm`              | Execution order for MRM comfortable stop              |
| `/system/mrm/pull_over_manager/operate` | `tier4_system_msgs::srv::OperateMrm`              | Execution order for MRM pull over                     |

## Parameters

{{ json_to_markdown("system/mrm_handler/schema/mrm_handler.schema.json") }}

## Assumptions / Known limits

TBD.
