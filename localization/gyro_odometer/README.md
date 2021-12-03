# gyro_odometer

## Purpose

`gyro_odometer` is the package to estimate twist by combining imu and vehicle speed.

## Inputs / Outputs

### Input

| Name                            | Type                                             | Description                        |
| ------------------------------- | ------------------------------------------------ | ---------------------------------- |
| `vehicle/twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | twist with covariance from vehicle |
| `imu`                           | `sensor_msgs::msg::Imu`                          | imu from sensor                    |

### Output

| Name                    | Type                                             | Description                     |
| ----------------------- | ------------------------------------------------ | ------------------------------- |
| `twist`                 | `geometry_msgs::msg::TwistStamped`               | estimated twist                 |
| `twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | estimated twist with covariance |

## Parameters

| Parameter      | Type   | Description       |
| -------------- | ------ | ----------------- |
| `output_frame` | String | output's frame id |

## Assumptions / Known limits

TBD.
