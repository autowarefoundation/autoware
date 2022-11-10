# mrm_emergency_stop_operator

## Purpose

MRM emergency stop operator is a node that generates emergency stop commands according to the emergency stop MRM order.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                                 | Type                                                       | Description                                                                                                                   |
| ------------------------------------ | ---------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `~/input/mrm/emergency_stop/operate` | `tier4_system_msgs::srv::OperateMrm`                       | MRM execution order                                                                                                           |
| `~/input/control/control_cmd`        | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command output from the last node of the control component. Used for the initial value of the emergency stop command. |
|                                      |                                                            |                                                                                                                               |

### Output

| Name                                      | Type                                                       | Description            |
| ----------------------------------------- | ---------------------------------------------------------- | ---------------------- |
| `~/output/mrm/emergency_stop/status`      | `tier4_system_msgs::msg::MrmBehaviorStatus`                | MRM execution status   |
| `~/output/mrm/emergency_stop/control_cmd` | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Emergency stop command |

## Parameters

### Node Parameters

| Name        | Type | Default value | Explanation                   |
| ----------- | ---- | ------------- | ----------------------------- |
| update_rate | int  | `30`          | Timer callback frequency [Hz] |

### Core Parameters

| Name                | Type   | Default value | Explanation                                    |
| ------------------- | ------ | ------------- | ---------------------------------------------- |
| target_acceleration | double | `-2.5`        | Target acceleration for emergency stop [m/s^2] |
| target_jerk         | double | `-1.5`        | Target jerk for emergency stop [m/s^3]         |

## Assumptions / Known limits

TBD.
