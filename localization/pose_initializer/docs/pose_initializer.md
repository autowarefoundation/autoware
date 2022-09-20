# pose_initializer

## Purpose

`pose_initializer` is the package to send an initial pose to `ekf_localizer`.
It receives roughly estimated initial pose from GNSS/user.
Passing the pose to `ndt_scan_matcher`, and it gets a calculated ego pose from `ndt_scan_matcher` via service.
Finally, it publishes the initial pose to `ekf_localizer`.

## Interfaces

### Parameters

| Name                  | Type | Description                                                                              |
| --------------------- | ---- | ---------------------------------------------------------------------------------------- |
| `ndt_enabled`         | bool | If true, the pose will be estimated by NDT scan matcher, otherwise it is passed through. |
| `stop_check_enabled`  | bool | If true, initialization is accepted only when the vehicle is stopped.                    |
| `stop_check_duration` | bool | The duration used for the stop check above.                                              |
| `gnss_enabled`        | bool | If true, use the GNSS pose when no pose is specified.                                    |
| `gnss_pose_timeout`   | bool | The duration that the GNSS pose is valid.                                                |

### Services

| Name                       | Type                                                | Description           |
| -------------------------- | --------------------------------------------------- | --------------------- |
| `/localization/initialize` | autoware_adapi_v1_msgs::srv::InitializeLocalization | initial pose from api |

### Clients

| Name                                         | Type                                                    | Description             |
| -------------------------------------------- | ------------------------------------------------------- | ----------------------- |
| `/localization/pose_estimator/ndt_align_srv` | tier4_localization_msgs::srv::PoseWithCovarianceStamped | pose estimation service |

### Subscriptions

| Name                                                        | Type                                          | Description          |
| ----------------------------------------------------------- | --------------------------------------------- | -------------------- |
| `/sensing/gnss/pose_with_covariance`                        | geometry_msgs::msg::PoseWithCovarianceStamped | pose from gnss       |
| `/sensing/vehicle_velocity_converter/twist_with_covariance` | geometry_msgs::msg::TwistStamped              | twist for stop check |

### Publications

| Name                                 | Type                                                         | Description                 |
| ------------------------------------ | ------------------------------------------------------------ | --------------------------- |
| `/localization/initialization_state` | autoware_adapi_v1_msgs::msg::LocalizationInitializationState | pose initialization state   |
| `/initialpose3d`                     | geometry_msgs::msg::PoseWithCovarianceStamped                | calculated initial ego pose |
