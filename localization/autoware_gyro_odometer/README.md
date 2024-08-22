# autoware_gyro_odometer

## Purpose

`autoware_gyro_odometer` is the package to estimate twist by combining imu and vehicle speed.

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

{{ json_to_markdown("localization/autoware_gyro_odometer/schema/gyro_odometer.schema.json") }}

## Assumptions / Known limits

- [Assumption] The frame_id of input twist message must be set to base_link.

- [Assumption] The covariance in the input messages must be properly assigned.

- [Assumption] The angular velocity is set to zero if both the longitudinal vehicle velocity and the angular velocity around the yaw axis are sufficiently small. This is for suppression of the IMU angular velocity bias. Without this process, we misestimate the vehicle status when stationary.

- [Limitation] The frequency of the output messages depends on the frequency of the input IMU message.

- [Limitation] We cannot produce reliable values for the lateral and vertical velocities. Therefore we assign large values to the corresponding elements in the output covariance matrix.

## Diagnostics

<img src="./media/diagnostic.png" alt="drawing" width="600"/>

| Name                             | Description                                                                               | Transition condition to Warning | Transition condition to Error                     |
| -------------------------------- | ----------------------------------------------------------------------------------------- | ------------------------------- | ------------------------------------------------- |
| `topic_time_stamp`               | the time stamp of service calling. [nano second]                                          | none                            | none                                              |
| `is_arrived_first_vehicle_twist` | whether the vehicle twist topic has been received even once.                              | not arrive yet                  | none                                              |
| `is_arrived_first_imu`           | whether the imu topic has been received even once.                                        | not arrive yet                  | none                                              |
| `vehicle_twist_time_stamp_dt`    | the time difference between the current time and the latest vehicle twist topic. [second] | none                            | the time is **longer** than `message_timeout_sec` |
| `imu_time_stamp_dt`              | the time difference between the current time and the latest imu topic. [second]           | none                            | the time is **longer** than `message_timeout_sec` |
| `vehicle_twist_queue_size`       | the size of vehicle_twist_queue.                                                          | none                            | none                                              |
| `imu_queue_size`                 | the size of gyro_queue.                                                                   | none                            | none                                              |
| `is_succeed_transform_imu`       | whether transform imu is succeed or not.                                                  | none                            | failed                                            |
