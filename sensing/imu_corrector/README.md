# imu_corrector

## Purpose

`imu_corrector_node` is a node that correct imu data.

1. Correct yaw rate offset by reading the parameter.
2. Correct yaw rate standard deviation by reading the parameter.

Use the value estimated by [deviation_estimator](https://github.com/tier4/calibration_tools/tree/main/localization/deviation_estimation_tools) as the parameters for this node.

## Inputs / Outputs

### Input

| Name     | Type                    | Description  |
| -------- | ----------------------- | ------------ |
| `~input` | `sensor_msgs::msg::Imu` | raw imu data |

### Output

| Name      | Type                    | Description        |
| --------- | ----------------------- | ------------------ |
| `~output` | `sensor_msgs::msg::Imu` | corrected imu data |

## Parameters

### Core Parameters

| Name                         | Type   | Description                         |
| ---------------------------- | ------ | ----------------------------------- |
| `angular_velocity_offset_z`  | double | yaw rate offset [rad/s]             |
| `angular_velocity_stddev_zz` | double | yaw rate standard deviation [rad/s] |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
