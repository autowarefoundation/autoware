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
| `twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | estimated twist with covariance |

## Parameters

| Parameter             | Type   | Description                      |
| --------------------- | ------ | -------------------------------- |
| `output_frame`        | String | output's frame id                |
| `message_timeout_sec` | Double | delay tolerance time for message |

## Assumptions / Known limits

- [Assumption] The frame_id of input twist message must be set to base_link.

- [Assumption] The covariance in the input messages must be properly assigned.

- [Assumption] The angular velocity is set to zero if both the longitudinal vehicle velocity and the angular velocity around the yaw axis are sufficiently small. This is for suppression of the IMU angular velocity bias. Without this process, we misestimate the vehicle status when stationary.

- [Limitation] The frequency of the output messages depends on the frequency of the input IMU message.

- [Limitation] We cannot produce reliable values for the lateral and vertical velocities. Therefore we assign large values to the corresponding elements in the output covariance matrix.
