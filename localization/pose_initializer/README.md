# pose_initializer

## Purpose

The `pose_initializer` is the package to send an initial pose to `ekf_localizer`.
It receives roughly estimated initial pose from GNSS/user.
Passing the pose to `ndt_scan_matcher`, and it gets a calculated ego pose from `ndt_scan_matcher` via service.
Finally, it publishes the initial pose to `ekf_localizer`.
This node depends on the map height fitter library.
[See here for more details.](../../map/map_height_fitter/README.md)

## Interfaces

### Parameters

| Name                  | Type | Description                                                                              |
| --------------------- | ---- | ---------------------------------------------------------------------------------------- |
| `ekf_enabled`         | bool | If true, EKF localizar is activated.                                                     |
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

## Connection with Default AD API

This `pose_initializer` is used via default AD API. For detailed description of the API description, please refer to [the description of `default_ad_api`](https://github.com/autowarefoundation/autoware.universe/blob/main/system/default_ad_api/document/localization.md).

<img src="../../system/default_ad_api/document/images/localization.drawio.svg" alt="drawing" width="800"/>
