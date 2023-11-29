# raw_vehicle_cmd_converter

`raw_vehicle_command_converter` is a node that converts desired acceleration and velocity to mechanical input by using feed forward + feed back control (optional).

## Input topics

| Name                  | Type                                                     | Description                                                                                                        |
| --------------------- | -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `~/input/control_cmd` | autoware_auto_control_msgs::msg::AckermannControlCommand | target `velocity/acceleration/steering_angle/steering_angle_velocity` is necessary to calculate actuation command. |
| `~/input/steering"`   | autoware_auto_vehicle_msgs::SteeringReport               | current status of steering used for steering feed back control                                                     |
| `~/input/twist`       | navigation_msgs::Odometry                                | twist topic in odometry is used.                                                                                   |

## Output topics

| Name                     | Type                                             | Description                                             |
| ------------------------ | ------------------------------------------------ | ------------------------------------------------------- |
| `~/output/actuation_cmd` | tier4_vehicle_msgs::msg::ActuationCommandStamped | actuation command for vehicle to apply mechanical input |

## Parameters

{{ json_to_markdown("vehicle/raw_vehicle_cmd_converter/schema/raw_vehicle_cmd_converter.schema.json") }}

## Limitation

The current feed back implementation is only applied to steering control.
