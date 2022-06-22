# twist2accel

## Purpose

This package is responsible for estimating acceleration using the output of `ekf_localizer`. It uses lowpass filter to mitigate the noise.

## Inputs / Outputs

### Input

| Name          | Type                                             | Description           |
| ------------- | ------------------------------------------------ | --------------------- |
| `input/odom`  | `nav_msgs::msg::Odometry`                        | localization odometry |
| `input/twist` | `geometry_msgs::msg::TwistWithCovarianceStamped` | twist                 |

### Output

| Name           | Type                                             | Description            |
| -------------- | ------------------------------------------------ | ---------------------- |
| `output/accel` | `geometry_msgs::msg::AccelWithCovarianceStamped` | estimated acceleration |

## Parameters

| Name                 | Type   | Description                                                               |
| -------------------- | ------ | ------------------------------------------------------------------------- |
| `use_odom`           | bool   | use odometry if true, else use twist input (default: true)                |
| `accel_lowpass_gain` | double | lowpass gain for lowpass filter in estimating acceleration (default: 0.9) |

## Future work

Future work includes integrating acceleration into the EKF state.
