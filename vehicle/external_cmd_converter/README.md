# external_cmd_converter

`external_cmd_converter` is a node that converts desired mechanical input to acceleration and velocity by using accel/brake map.

## Input topics

| Name                        | Type                                         | Description                                                                                                       |
| --------------------------- | -------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `~/in/external_control_cmd` | tier4_external_api_msgs::msg::ControlCommand | target `throttle/brake/steering_angle/steering_angle_velocity` is necessary to calculate desired control command. |
| `~/input/shift_cmd"`        | autoware_auto_vehicle_msgs::GearCommand      | current command of gear.                                                                                          |
| `~/input/emergency_stop`    | tier4_external_api_msgs::msg::Heartbeat      | emergency heart beat for external command.                                                                        |
| `~/input/current_gate_mode` | tier4_control_msgs::msg::GateMode            | topic for gate mode.                                                                                              |
| `~/input/odometry`          | navigation_msgs::Odometry                    | twist topic in odometry is used.                                                                                  |

## Output topics

| Name                | Type                                                     | Description                                                        |
| ------------------- | -------------------------------------------------------- | ------------------------------------------------------------------ |
| `~/out/control_cmd` | autoware_auto_control_msgs::msg::AckermannControlCommand | ackermann control command converted from selected external command |

## Parameters

| Parameter                 | Type   | Description                                           |
| ------------------------- | ------ | ----------------------------------------------------- |
| `timer_rate`              | double | timer's update rate                                   |
| `wait_for_first_topic`    | double | if time out check is done after receiving first topic |
| `control_command_timeout` | double | time out check for control command                    |
| `emergency_stop_timeout`  | double | time out check for emergency stop command             |

## Limitation

tbd.
