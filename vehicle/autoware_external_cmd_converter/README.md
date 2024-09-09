# external_cmd_converter

`external_cmd_converter` is a node that converts desired mechanical input to acceleration and velocity by using accel/brake map.

## Algorithm

### How to calculate reference acceleration and velocity

A reference acceleration and velocity are derived from the throttle and brake values of external control commands.

#### Reference Acceleration

A reference acceleration is calculated from accel_brake_map based on values of a desired_pedal and a current velocity;

$$
    pedal_d = throttle_d - brake_d,
$$

$$
    acc_{ref} = Acc(pedal_d, v_{x,current}).
$$

| Parameter       | Description                                                                               |
| --------------- | ----------------------------------------------------------------------------------------- |
| $throttle_d$    | throttle value of external control command (`~/in/external_control_cmd.control.throttle`) |
| $brake_d$       | brake value of external control command (`~/in/external_control_cmd.control.brake`)       |
| $v_{x,current}$ | current longitudinal velocity (`~/in/odometry.twist.twist.linear.x`)                      |
| Acc             | accel_brake_map                                                                           |

#### Reference Velocity

A reference velocity is calculated based on a current velocity and a reference acceleration:

$$
v_{ref} =
    v_{x,current} + k_{v_{ref}} \cdot \text{sign}_{gear} \cdot acc_{ref}.
$$

| Parameter            | Description                                                           |
| -------------------- | --------------------------------------------------------------------- |
| $acc_{ref}$          | reference acceleration                                                |
| $k_{v_{ref}}$        | reference velocity gain                                               |
| $\text{sign}_{gear}$ | gear command (`~/in/shift_cmd`) (Drive/Low: 1, Reverse: -1, Other: 0) |

## Input topics

| Name                        | Type                                         | Description                                                                                                       |
| --------------------------- | -------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `~/in/external_control_cmd` | tier4_external_api_msgs::msg::ControlCommand | target `throttle/brake/steering_angle/steering_angle_velocity` is necessary to calculate desired control command. |
| `~/input/shift_cmd"`        | autoware_vehicle_msgs::GearCommand           | current command of gear.                                                                                          |
| `~/input/emergency_stop`    | tier4_external_api_msgs::msg::Heartbeat      | emergency heart beat for external command.                                                                        |
| `~/input/current_gate_mode` | tier4_control_msgs::msg::GateMode            | topic for gate mode.                                                                                              |
| `~/input/odometry`          | navigation_msgs::Odometry                    | twist topic in odometry is used.                                                                                  |

## Output topics

| Name                | Type                                | Description                                                        |
| ------------------- | ----------------------------------- | ------------------------------------------------------------------ |
| `~/out/control_cmd` | autoware_control_msgs::msg::Control | ackermann control command converted from selected external command |

## Parameters

| Parameter                 | Type   | Description                                           |
| ------------------------- | ------ | ----------------------------------------------------- |
| `ref_vel_gain_`           | double | reference velocity gain                               |
| `timer_rate`              | double | timer's update rate                                   |
| `wait_for_first_topic`    | double | if time out check is done after receiving first topic |
| `control_command_timeout` | double | time out check for control command                    |
| `emergency_stop_timeout`  | double | time out check for emergency stop command             |

## Limitation

tbd.
