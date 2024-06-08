# autoware_joy_controller

## Role

`autoware_joy_controller` is the package to convert a joy msg to autoware commands (e.g. steering wheel, shift, turn signal, engage) for a vehicle.

## Usage

### ROS 2 launch

```bash
# With default config (ds4)
ros2 launch autoware_joy_controller joy_controller.launch.xml

# Default config but select from the existing parameter files
ros2 launch autoware_joy_controller joy_controller_param_selection.launch.xml joy_type:=ds4 # or g29, p65, xbox

# Override the param file
ros2 launch autoware_joy_controller joy_controller.launch.xml config_file:=/path/to/your/param.yaml
```

## Input / Output

### Input topics

| Name               | Type                    | Description                       |
| ------------------ | ----------------------- | --------------------------------- |
| `~/input/joy`      | sensor_msgs::msg::Joy   | joy controller command            |
| `~/input/odometry` | nav_msgs::msg::Odometry | ego vehicle odometry to get twist |

### Output topics

| Name                                | Type                                                | Description                              |
| ----------------------------------- | --------------------------------------------------- | ---------------------------------------- |
| `~/output/control_command`          | autoware_control_msgs::msg::Control                 | lateral and longitudinal control command |
| `~/output/external_control_command` | tier4_external_api_msgs::msg::ControlCommandStamped | lateral and longitudinal control command |
| `~/output/shift`                    | tier4_external_api_msgs::msg::GearShiftStamped      | gear command                             |
| `~/output/turn_signal`              | tier4_external_api_msgs::msg::TurnSignalStamped     | turn signal command                      |
| `~/output/gate_mode`                | tier4_control_msgs::msg::GateMode                   | gate mode (Auto or External)             |
| `~/output/heartbeat`                | tier4_external_api_msgs::msg::Heartbeat             | heartbeat                                |
| `~/output/vehicle_engage`           | autoware_vehicle_msgs::msg::Engage                  | vehicle engage                           |

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
| `raw_control`             | bool   | skip input odometry if true                                                                                        |
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

## XBOX Joystick Key Map

| Action               | Button                |
| -------------------- | --------------------- |
| Acceleration         | RT                    |
| Brake                | LT                    |
| Steering             | Left Stick Left Right |
| Shift up             | Cursor Up             |
| Shift down           | Cursor Down           |
| Shift Drive          | Cursor Left           |
| Shift Reverse        | Cursor Right          |
| Turn Signal Left     | LB                    |
| Turn Signal Right    | RB                    |
| Clear Turn Signal    | A                     |
| Gate Mode            | B                     |
| Emergency Stop       | View                  |
| Clear Emergency Stop | Menu                  |
| Autoware Engage      | X                     |
| Autoware Disengage   | Y                     |
| Vehicle Engage       | Left Stick Button     |
| Vehicle Disengage    | Right Stick Button    |
