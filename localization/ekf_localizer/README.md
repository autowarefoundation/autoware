# Overview

The **Extend Kalman Filter Localizer** estimates robust and less noisy robot pose and twist by integrating the 2D vehicle dynamics model with input ego-pose and ego-twist messages. The algorithm is designed especially for fast-moving robots such as autonomous driving systems.

## Flowchart

The overall flowchart of the ekf_localizer is described below.

<p align="center">
  <img src="./media/ekf_flowchart.png" width="800">
</p>

## Features

This package includes the following features:

- **Time delay compensation** for input messages, which enables proper integration of input information with varying time delays. This is important especially for high-speed moving robots, such as autonomous driving vehicles. (see the following figure).
- **Automatic estimation of yaw bias** prevents modeling errors caused by sensor mounting angle errors, which can improve estimation accuracy.
- **Mahalanobis distance gate** enables probabilistic outlier detection to determine which inputs should be used or ignored.
- **Smooth update**, the Kalman Filter measurement update is typically performed when a measurement is obtained, but it can cause large changes in the estimated value, especially for low-frequency measurements. Since the algorithm can consider the measurement time, the measurement data can be divided into multiple pieces and integrated smoothly while maintaining consistency (see the following figure).
- **Calculation of vertical correction amount from pitch** mitigates localization instability on slopes. For example, when going uphill, it behaves as if it is buried in the ground (see the left side of the "Calculate delta from pitch" figure) because EKF only considers 3DoF(x,y,yaw). Therefore, EKF corrects the z-coordinate according to the formula (see the right side of the "Calculate delta from pitch" figure).

<p align="center">
<img src="./media/ekf_delay_comp.png" width="800">
</p>

<p align="center">
  <img src="./media/ekf_smooth_update.png" width="800">
</p>

<p align="center">
  <img src="./media/calculation_delta_from_pitch.png" width="800">
</p>

## Node

### Subscribed Topics

| Name                             | Type                                             | Description                                                                                                                              |
| -------------------------------- | ------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `measured_pose_with_covariance`  | `geometry_msgs::msg::PoseWithCovarianceStamped`  | Input pose source with the measurement covariance matrix.                                                                                |
| `measured_twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | Input twist source with the measurement covariance matrix.                                                                               |
| `initialpose`                    | `geometry_msgs::msg::PoseWithCovarianceStamped`  | Initial pose for EKF. The estimated pose is initialized with zeros at the start. It is initialized with this message whenever published. |

### Published Topics

| Name                              | Type                                             | Description                                           |
| --------------------------------- | ------------------------------------------------ | ----------------------------------------------------- |
| `ekf_odom`                        | `nav_msgs::msg::Odometry`                        | Estimated odometry.                                   |
| `ekf_pose`                        | `geometry_msgs::msg::PoseStamped`                | Estimated pose.                                       |
| `ekf_pose_with_covariance`        | `geometry_msgs::msg::PoseWithCovarianceStamped`  | Estimated pose with covariance.                       |
| `ekf_biased_pose`                 | `geometry_msgs::msg::PoseStamped`                | Estimated pose including the yaw bias                 |
| `ekf_biased_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped`  | Estimated pose with covariance including the yaw bias |
| `ekf_twist`                       | `geometry_msgs::msg::TwistStamped`               | Estimated twist.                                      |
| `ekf_twist_with_covariance`       | `geometry_msgs::msg::TwistWithCovarianceStamped` | The estimated twist with covariance.                  |
| `diagnostics`                     | `diagnostics_msgs::msg::DiagnosticArray`         | The diagnostic information.                           |

### Published TF

- base_link
  TF from `map` coordinate to estimated pose.

## Functions

### Predict

The current robot state is predicted from previously estimated data using a given prediction model. This calculation is called at a constant interval (`predict_frequency [Hz]`). The prediction equation is described at the end of this page.

### Measurement Update

Before the update, the Mahalanobis distance is calculated between the measured input and the predicted state, the measurement update is not performed for inputs where the Mahalanobis distance exceeds the given threshold.

The predicted state is updated with the latest measured inputs, measured_pose, and measured_twist. The updates are performed with the same frequency as prediction, usually at a high frequency, in order to enable smooth state estimation.

## Parameter description

The parameters are set in `launch/ekf_localizer.launch` .

### For Node

{{ json_to_markdown("localization/ekf_localizer/schema/sub/node.sub_schema.json") }}

### For pose measurement

{{ json_to_markdown("localization/ekf_localizer/schema/sub/pose_measurement.sub_schema.json") }}

### For twist measurement

{{ json_to_markdown("localization/ekf_localizer/schema/sub/twist_measurement.sub_schema.json") }}

### For process noise

{{ json_to_markdown("localization/ekf_localizer/schema/sub/process_noise.sub_schema.json") }}

note: process noise for positions x & y are calculated automatically from nonlinear dynamics.

### Simple 1D Filter Parameters

{{ json_to_markdown("localization/ekf_localizer/schema/sub/simple_1d_filter_parameters.sub_schema.json") }}

### For diagnostics

{{ json_to_markdown("localization/ekf_localizer/schema/sub/diagnostics.sub_schema.json") }}

### Misc

{{ json_to_markdown("localization/ekf_localizer/schema/sub/misc.sub_schema.json") }}

## How to tune EKF parameters

### 0. Preliminaries

- Check header time in pose and twist message is set to sensor time appropriately, because time delay is calculated from this value. If it is difficult to set an appropriate time due to the timer synchronization problem, use `twist_additional_delay` and `pose_additional_delay` to correct the time.
- Check if the relation between measurement pose and twist is appropriate (whether the derivative of the pose has a similar value to twist). This discrepancy is caused mainly by unit error (such as confusing radian/degree) or bias noise, and it causes large estimation errors.

### 1. Tune sensor parameters

Set standard deviation for each sensor. The `pose_measure_uncertainty_time` is for the uncertainty of the header timestamp data.
You can also tune a number of steps for smoothing for each observed sensor data by tuning `*_smoothing_steps`.
Increasing the number will improve the smoothness of the estimation, but may have an adverse effect on the estimation performance.

- `pose_measure_uncertainty_time`
- `pose_smoothing_steps`
- `twist_smoothing_steps`

### 2. Tune process model parameters

- `proc_stddev_vx_c` : set to maximum linear acceleration
- `proc_stddev_wz_c` : set to maximum angular acceleration
- `proc_stddev_yaw_c` : This parameter describes the correlation between the yaw and yaw rate. A large value means the change in yaw does not correlate to the estimated yaw rate. If this is set to 0, it means the change in estimated yaw is equal to yaw rate. Usually, this should be set to 0.
- `proc_stddev_yaw_bias_c` : This parameter is the standard deviation for the rate of change in yaw bias. In most cases, yaw bias is constant, so it can be very small, but must be non-zero.

## Kalman Filter Model

### kinematics model in update function

<img src="./media/ekf_dynamics.png" width="320">

where, $\theta_k$ represents the vehicle's heading angle, including the mounting angle bias.
$b_k$ is a correction term for the yaw bias, and it is modeled so that $(\theta_k+b_k)$ becomes the heading angle of the base_link.
The pose_estimator is expected to publish the base_link in the map coordinate system. However, the yaw angle may be offset due to calibration errors. This model compensates this error and improves estimation accuracy.

### time delay model

The measurement time delay is handled by an augmented state [1] (See, Section 7.3 FIXED-LAG SMOOTHING).

<img src="./media/delay_model_eq.png" width="320">

Note that, although the dimension gets larger since the analytical expansion can be applied based on the specific structures of the augmented states, the computational complexity does not significantly change.

## Test Result with Autoware NDT

<p align="center">
<img src="./media/ekf_autoware_res.png" width="600">
</p>

## Diagnostics

<p align="center">
<img src="./media/ekf_diagnostics.png" width="320">
</p>

### The conditions that result in a WARN state

- The node is not in the activate state.
- The number of consecutive no measurement update via the Pose/Twist topic exceeds the `pose_no_update_count_threshold_warn`/`twist_no_update_count_threshold_warn`.
- The timestamp of the Pose/Twist topic is beyond the delay compensation range.
- The Pose/Twist topic is beyond the range of Mahalanobis distance for covariance estimation.

### The conditions that result in an ERROR state

- The number of consecutive no measurement update via the Pose/Twist topic exceeds the `pose_no_update_count_threshold_error`/`twist_no_update_count_threshold_error`.

## Known issues

- If multiple pose_estimators are used, the input to the EKF will include multiple yaw biases corresponding to each source. However, the current EKF assumes the existence of only one yaw bias. Therefore, yaw bias `b_k` in the current EKF state would not make any sense and cannot correctly handle these multiple yaw biases. Thus, future work includes introducing yaw bias for each sensor with yaw estimation.

## reference

[1] Anderson, B. D. O., & Moore, J. B. (1979). Optimal filtering. Englewood Cliffs, NJ: Prentice-Hall.
