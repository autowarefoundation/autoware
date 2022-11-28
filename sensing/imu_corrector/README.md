# imu_corrector

## Purpose

`imu_corrector_node` is a node that correct imu data.

1. Correct yaw rate offset $b$ by reading the parameter.
2. Correct yaw rate standard deviation $\sigma$ by reading the parameter.

Mathematically, we assume the following equation:

$$
\tilde{\omega}(t) = \omega(t) + b(t) + n(t)
$$

where $\tilde{\omega}$ denotes observed angular velocity, $\omega$ denotes true angular velocity, $b$ denotes an offset, and $n$ denotes a gaussian noise.
We also assume that $n\sim\mathcal{N}(0, \sigma^2)$.

<!-- TODO(TIER IV): Make this repository public or change the link. -->
<!-- Use the value estimated by [deviation_estimator](https://github.com/tier4/calibration_tools/tree/main/localization/deviation_estimation_tools) as the parameters for this node. -->

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

| Name                         | Type   | Description                                      |
| ---------------------------- | ------ | ------------------------------------------------ |
| `angular_velocity_offset_x`  | double | roll rate offset in imu_link [rad/s]             |
| `angular_velocity_offset_y`  | double | pitch rate offset imu_link [rad/s]               |
| `angular_velocity_offset_z`  | double | yaw rate offset imu_link [rad/s]                 |
| `angular_velocity_stddev_xx` | double | roll rate standard deviation imu_link [rad/s]    |
| `angular_velocity_stddev_yy` | double | pitch rate standard deviation imu_link [rad/s]   |
| `angular_velocity_stddev_zz` | double | yaw rate standard deviation imu_link [rad/s]     |
| `acceleration_stddev`        | double | acceleration standard deviation imu_link [m/s^2] |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
