# raw_vehicle_cmd_converter

## Overview

The raw_vehicle_command_converter is a crucial node in vehicle automation systems, responsible for translating desired steering and acceleration inputs into specific vehicle control commands. This process is achieved through a combination of a lookup table and an optional feedback control system.

### Lookup Table

The core of the converter's functionality lies in its use of a CSV-formatted lookup table. This table encapsulates the relationship between the throttle/brake pedal (depending on your vehicle control interface) and the corresponding vehicle acceleration across various speeds. The converter utilizes this data to accurately translate target accelerations into appropriate throttle/brake values.

![accel-brake-map-table](./figure/accel-brake-map-table.png)

### Creation of Reference Data

Reference data for the lookup table is generated through the following steps:

1. **Data Collection**: On a flat road, a constant value command (e.g., throttle/brake pedal) is applied to accelerate or decelerate the vehicle.
2. **Recording Data**: During this phase, both the IMU acceleration and vehicle velocity data are recorded.
3. **CSV File Generation**: A CSV file is created, detailing the relationship between command values, vehicle speed, and resulting acceleration.

Once the acceleration map is crafted, it should be loaded when the RawVehicleCmdConverter node is launched, with the file path defined in the launch file.

### Auto-Calibration Tool

For ease of calibration and adjustments to the lookup table, an auto-calibration tool is available. More information and instructions for this tool can be found [here](https://github.com/autowarefoundation/autoware.universe/blob/main/vehicle/accel_brake_map_calibrator/accel_brake_map_calibrator/README.md).

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
