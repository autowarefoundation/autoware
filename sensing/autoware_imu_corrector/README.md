# autoware_imu_corrector

## imu_corrector

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

### Input

| Name     | Type                    | Description  |
| -------- | ----------------------- | ------------ |
| `~input` | `sensor_msgs::msg::Imu` | raw imu data |

### Output

| Name      | Type                    | Description        |
| --------- | ----------------------- | ------------------ |
| `~output` | `sensor_msgs::msg::Imu` | corrected imu data |

### Parameters

| Name                         | Type   | Description                                      |
| ---------------------------- | ------ | ------------------------------------------------ |
| `angular_velocity_offset_x`  | double | roll rate offset in imu_link [rad/s]             |
| `angular_velocity_offset_y`  | double | pitch rate offset imu_link [rad/s]               |
| `angular_velocity_offset_z`  | double | yaw rate offset imu_link [rad/s]                 |
| `angular_velocity_stddev_xx` | double | roll rate standard deviation imu_link [rad/s]    |
| `angular_velocity_stddev_yy` | double | pitch rate standard deviation imu_link [rad/s]   |
| `angular_velocity_stddev_zz` | double | yaw rate standard deviation imu_link [rad/s]     |
| `acceleration_stddev`        | double | acceleration standard deviation imu_link [m/s^2] |

## gyro_bias_estimator

`gyro_bias_validator` is a node that validates the bias of the gyroscope. It subscribes to the `sensor_msgs::msg::Imu` topic and validate if the bias of the gyroscope is within the specified range.

Note that the node calculates bias from the gyroscope data by averaging the data only when the vehicle is stopped.

### Input

| Name              | Type                                            | Description      |
| ----------------- | ----------------------------------------------- | ---------------- |
| `~/input/imu_raw` | `sensor_msgs::msg::Imu`                         | **raw** imu data |
| `~/input/pose`    | `geometry_msgs::msg::PoseWithCovarianceStamped` | ndt pose         |

Note that the input pose is assumed to be accurate enough. For example when using NDT, we assume that the NDT is appropriately converged.

Currently, it is possible to use methods other than NDT as a `pose_source` for Autoware, but less accurate methods are not suitable for IMU bias estimation.

In the future, with careful implementation for pose errors, the IMU bias estimated by NDT could potentially be used not only for validation but also for online calibration.

### Output

| Name                 | Type                                 | Description                   |
| -------------------- | ------------------------------------ | ----------------------------- |
| `~/output/gyro_bias` | `geometry_msgs::msg::Vector3Stamped` | bias of the gyroscope [rad/s] |

### Parameters

Note that this node also uses `angular_velocity_offset_x`, `angular_velocity_offset_y`, `angular_velocity_offset_z` parameters from `imu_corrector.param.yaml`.

| Name                                  | Type   | Description                                                                                 |
| ------------------------------------- | ------ | ------------------------------------------------------------------------------------------- |
| `gyro_bias_threshold`                 | double | threshold of the bias of the gyroscope [rad/s]                                              |
| `timer_callback_interval_sec`         | double | seconds about the timer callback function [sec]                                             |
| `diagnostics_updater_interval_sec`    | double | period of the diagnostics updater [sec]                                                     |
| `straight_motion_ang_vel_upper_limit` | double | upper limit of yaw angular velocity, beyond which motion is not considered straight [rad/s] |
