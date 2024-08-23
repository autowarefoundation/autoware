# pose_initializer

## Purpose

The `pose_initializer` is the package to send an initial pose to `ekf_localizer`.
It receives roughly estimated initial pose from GNSS/user.
Passing the pose to `ndt_scan_matcher`, and it gets a calculated ego pose from `ndt_scan_matcher` via service.
Finally, it publishes the initial pose to `ekf_localizer`.
This node depends on the map height fitter library.
[See here for more details.](../../map/autoware_map_height_fitter/README.md)

## Interfaces

### Parameters

{{ json_to_markdown("localization/pose_initializer/schema/pose_initializer.schema.json") }}

### Services

| Name                       | Type                                                 | Description           |
| -------------------------- | ---------------------------------------------------- | --------------------- |
| `/localization/initialize` | tier4_localization_msgs::srv::InitializeLocalization | initial pose from api |

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

## Diagnostics

### pose_initializer_status

If the score of initial pose estimation result is lower than score threshold, ERROR message is output to the `/diagnostics` topic.

<img src="./media/diagnostic_pose_reliability.png" alt="drawing" width="400"/>

## Connection with Default AD API

This `pose_initializer` is used via default AD API. For detailed description of the API description, please refer to [the description of `default_ad_api`](https://github.com/autowarefoundation/autoware.universe/blob/main/system/default_ad_api/document/localization.md).

<img src="../../system/default_ad_api/document/images/localization.drawio.svg" alt="drawing" width="800"/>

## Initialize pose via CLI

### Using the GNSS estimated position

```bash
ros2 service call /localization/initialize tier4_localization_msgs/srv/InitializeLocalization
```

The GNSS estimated position is used as the initial guess, and the localization algorithm automatically estimates a more accurate position.

### Using the input position

```bash
ros2 service call /localization/initialize tier4_localization_msgs/srv/InitializeLocalization "
pose_with_covariance:
  - header:
      frame_id: map
    pose:
      pose:
        position:
          x: 89571.1640625
          y: 42301.1875
          z: -3.1565165519714355
        orientation:
          x: 0.0
          y: 0.0
          z: 0.28072773940524687
          w: 0.9597874433062874
      covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.06853891909122467]
method: 0
"
```

The input position is used as the initial guess, and the localization algorithm automatically estimates a more accurate position.

### Direct initial position set

```bash
ros2 service call /localization/initialize tier4_localization_msgs/srv/InitializeLocalization "
pose_with_covariance:
  - header:
      frame_id: map
    pose:
      pose:
        position:
          x: 89571.1640625
          y: 42301.1875
          z: -3.1565165519714355
        orientation:
          x: 0.0
          y: 0.0
          z: 0.28072773940524687
          w: 0.9597874433062874
      covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.06853891909122467]
method: 1
"
```

The initial position is set directly by the input position without going through localization algorithm.

### Via ros2 topic pub

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: map
pose:
  pose:
    position:
      x: 89571.1640625
      y: 42301.1875
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.28072773940524687
      w: 0.9597874433062874
"
```

It behaves the same as "initialpose (from rviz)".
The position.z and the covariance will be overwritten by [ad_api_adaptors](https://github.com/autowarefoundation/autoware.universe/tree/main/system/default_ad_api_helpers/ad_api_adaptors), so there is no need to input them.
