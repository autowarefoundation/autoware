# joy_controller

## Role

`joy_controller` is the package to convert a joy msg to autoware commands (e.g. steering wheel, shift, turn signal, engage) for a vehicle.

## Input / Output

### Input topics

| Name               | Type                    | Description                       |
| ------------------ | ----------------------- | --------------------------------- |
| `~/input/joy`      | sensor_msgs::msg::Joy   | joy controller command            |
| `~/input/odometry` | nav_msgs::msg::Odometry | ego vehicle odometry to get twist |

### Output topics

| Name                                | Type                                                     | Description                              |
| ----------------------------------- | -------------------------------------------------------- | ---------------------------------------- |
| `~/output/control_command`          | autoware_auto_control_msgs::msg::AckermannControlCommand | lateral and longitudinal control command |
| `~/output/external_control_command` | tier4_external_api_msgs::msg::ControlCommandStamped      | lateral and longitudinal control command |
| `~/output/shift`                    | tier4_external_api_msgs::msg::GearShiftStamped           | gear command                             |
| `~/output/turn_signal`              | tier4_external_api_msgs::msg::TurnSignalStamped          | turn signal command                      |
| `~/output/gate_mode`                | tier4_control_msgs::msg::GateMode                        | gate mode (Auto or External)             |
| `~/output/heartbeat`                | tier4_external_api_msgs::msg::Heartbeat                  | heartbeat                                |
| `~/output/vehicle_engage`           | autoware_auto_vehicle_msgs::msg::Engage                  | vehicle engage                           |

## Parameters

| Parameter                 | Type   | Description                                                                                                        |
| ------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------ |
| `joy_type`                | string | joy controller type (default: DS4)                                                                                 |
| `update_rate`             | double | update rate to publish control commands                                                                            |
| `accel_ratio`             | double | ratio to calculate acceleration (commanded acceleration is ratio \* operation)                                     |
| `brake_ratio`             | double | ratio to calculate deceleration (commanded acceleration is -ratio \* operation)                                    |
| `steer_ratio`             | double | ratio to calculate deceleration (commanded steer is ratio \* operation)                                            |
| `steering_angle_velocity` | double | steering angle velocity for operation                                                                              |
| `accel_sensitivity`       | double | sensitivity to calculate acceleration for external API (commanded acceleration is pow(operation, 1 / sensitivity)) |
| `brake_sensitivity`       | double | sensitivity to calculate deceleration for external API (commanded acceleration is pow(operation, 1 / sensitivity)) |
| `velocity_gain`           | double | ratio to calculate velocity by acceleration                                                                        |
| `max_forward_velocity`    | double | absolute max velocity to go forward                                                                                |
| `max_backward_velocity`   | double | absolute max velocity to go backward                                                                               |
| `backward_accel_ratio`    | double | ratio to calculate deceleration (commanded acceleration is -ratio \* operation)                                    |

## P65 Joystick Key Map

| Action               | Button                |
| -------------------- | --------------------- |
| Acceleration         | R2                    |
| Brake                | L2                    |
| Steering             | Left Stick Left Right |
| Shift up             | Cursor Up             |
| Shift down           | Cursor Down           |
| Shift Drive          | Cursor Left           |
| Shift Reverse        | Cursor Right          |
| Turn Signal Left     | L1                    |
| Turn Signal Right    | R1                    |
| Clear Turn Signal    | A                     |
| Gate Mode            | B                     |
| Emergency Stop       | Select                |
| Clear Emergency Stop | Start                 |
| Autoware Engage      | X                     |
| Autoware Disengage   | Y                     |
| Vehicle Engage       | PS                    |
| Vehicle Disengage    | Right Trigger         |

## DS4 Joystick Key Map

| Action               | Button                     |
| -------------------- | -------------------------- |
| Acceleration         | R2, ×, or Right Stick Up   |
| Brake                | L2, □, or Right Stick Down |
| Steering             | Left Stick Left Right      |
| Shift up             | Cursor Up                  |
| Shift down           | Cursor Down                |
| Shift Drive          | Cursor Left                |
| Shift Reverse        | Cursor Right               |
| Turn Signal Left     | L1                         |
| Turn Signal Right    | R1                         |
| Clear Turn Signal    | SHARE                      |
| Gate Mode            | OPTIONS                    |
| Emergency Stop       | PS                         |
| Clear Emergency Stop | PS                         |
| Autoware Engage      | ○                          |
| Autoware Disengage   | ○                          |
| Vehicle Engage       | △                          |
| Vehicle Disengage    | △                          |
