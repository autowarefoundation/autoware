# vehicle_velocity_converter

## Purpose

This package converts autoware_vehicle_msgs::msg::VehicleReport message to geometry_msgs::msg::TwistWithCovarianceStamped for gyro odometer node.

## Inputs / Outputs

### Input

| Name              | Type                                        | Description      |
| ----------------- | ------------------------------------------- | ---------------- |
| `velocity_status` | `autoware_vehicle_msgs::msg::VehicleReport` | vehicle velocity |

### Output

| Name                    | Type                                             | Description                                        |
| ----------------------- | ------------------------------------------------ | -------------------------------------------------- |
| `twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | twist with covariance converted from VehicleReport |

## Parameters

| Name                         | Type   | Description                             |
| ---------------------------- | ------ | --------------------------------------- |
| `speed_scale_factor`         | double | speed scale factor (ideal value is 1.0) |
| `frame_id`                   | string | frame id for output message             |
| `velocity_stddev_xx`         | double | standard deviation for vx               |
| `angular_velocity_stddev_zz` | double | standard deviation for yaw rate         |
