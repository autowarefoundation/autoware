# Overview

The **Extend Kalman Filter Localizer** estimates robust and less noisy robot pose and twist by integrating the 2D vehicle dynamics model with input ego-pose and ego-twist messages. The algorithm is designed especially for fast moving robot such as autonomous driving system.

## Flowchart

The overall flowchart of the ekf_localizer is described below.

<p align="center">
  <img src="./media/ekf_flowchart.png" width="800">
</p>

## Features

This package includes the following features:

- **Time delay compensation** for input messages, which enables proper integration of input information with varying time delay. This is important especially for high speed moving robot, such as autonomous driving vehicle. (see following figure).
- **Automatic estimation of yaw bias** prevents modeling errors caused by sensor mounting angle errors, which can improve estimation accuracy.
- **Mahalanobis distance gate** enables probabilistic outlier detection to determine which inputs should be used or ignored.
- **Smooth update**, the Kalman Filter measurement update is typically performed when a measurement is obtained, but it can cause large changes in the estimated value especially for low frequency measurements. Since the algorithm can consider the measurement time, the measurement data can be divided into multiple pieces and integrated smoothly while maintaining consistency (see following figure).

<p align="center">
<img src="./media/ekf_delay_comp.png" width="800">
</p>

<p align="center">
  <img src="./media/ekf_smooth_update.png" width="800">
</p>

## Launch

The `ekf_localizer` starts with the default parameters with the following command.

```sh
roslaunch ekf_localizer ekf_localizer.launch
```

The parameters and input topic names can be set in the `ekf_localizer.launch` file.

## Node

### Subscribed Topics

- measured_pose_with_covariance (geometry_msgs/PoseWithCovarianceStamped)

  Input pose source with measurement covariance matrix.

- measured_twist_with_covariance (geometry_msgs/PoseWithCovarianceStamped)

  Input twist source with measurement covariance matrix.

- initialpose (geometry_msgs/PoseWithCovarianceStamped)

  Initial pose for EKF. The estimated pose is initialized with zeros at start. It is initialized with this message whenever published.

### Published Topics

- ekf_odom (nav_msgs/Odometry)

  Estimated odometry.

- ekf_pose (geometry_msgs/PoseStamped)

  Estimated pose.

- ekf_pose_with_covariance (geometry_msgs/PoseWithCovarianceStamped)

  Estimated pose with covariance.

- ekf_pose_with_covariance (geometry_msgs/PoseStamped)

  Estimated pose without yawbias effect.

- ekf_pose_with_covariance_without_yawbias (geometry_msgs/PoseWithCovarianceStamped)

  Estimated pose with covariance without yawbias effect.

- ekf_twist (geometry_msgs/TwistStamped)

  Estimated twist.

- ekf_twist_with_covariance (geometry_msgs/TwistWithCovarianceStamped)

  Estimated twist with covariance.

### Published TF

- base_link

  TF from "map" coordinate to estimated pose.

## Functions

### Predict

The current robot state is predicted from previously estimated data using a given prediction model. This calculation is called at constant interval (`predict_frequency [Hz]`). The prediction equation is described at the end of this page.

### Measurement Update

Before update, the Mahalanobis distance is calculated between the measured input and the predicted state, the measurement update is not performed for inputs where the Mahalanobis distance exceeds the given threshold.

The predicted state is updated with the latest measured inputs, measured_pose and measured_twist. The updates are performed with the same frequency as prediction, usually at a high frequency, in order to enable smooth state estimation.

## Parameter description

The parameters are set in `launch/ekf_localizer.launch` .

### For Node

| Name                       | Type   | Description                                                                               | Default value |
| :------------------------- | :----- | :---------------------------------------------------------------------------------------- | :------------ |
| show_debug_info            | bool   | Flag to display debug info                                                                | false         |
| predict_frequency          | double | Frequency for filtering and publishing [Hz]                                               | 50.0          |
| tf_rate                    | double | Frequency for tf broadcasting [Hz]                                                        | 10.0          |
| extend_state_step          | int    | Max delay step which can be dealt with in EKF. Large number increases computational cost. | 50            |
| enable_yaw_bias_estimation | bool   | Flag to enable yaw bias estimation                                                        | true          |

### For pose measurement

| Name                          | Type   | Description                                                       | Default value |
| :---------------------------- | :----- | :---------------------------------------------------------------- | :------------ |
| pose_additional_delay         | double | Additional delay time for pose measurement [s]                    | 0.0           |
| pose_measure_uncertainty_time | double | Measured time uncertainty used for covariance calculation [s]     | 0.01          |
| pose_rate                     | double | Approximated input pose rate used for covariance calculation [Hz] | 10.0          |
| pose_gate_dist                | double | Limit of Mahalanobis distance used for outliers detection         | 10000.0       |

### For twist measurement

| Name                   | Type   | Description                                                        | Default value |
| :--------------------- | :----- | :----------------------------------------------------------------- | :------------ |
| twist_additional_delay | double | Additional delay time for twist [s]                                | 0.0           |
| twist_rate             | double | Approximated input twist rate used for covariance calculation [Hz] | 10.0          |
| twist_gate_dist        | double | Limit of Mahalanobis distance used for outliers detection          | 10000.0       |

### For process noise

| Name                   | Type   | Description                                                                                                      | Default value |
| :--------------------- | :----- | :--------------------------------------------------------------------------------------------------------------- | :------------ |
| proc_stddev_vx_c       | double | Standard deviation of process noise in time differentiation expression of linear velocity x, noise for d_vx = 0  | 2.0           |
| proc_stddev_wz_c       | double | Standard deviation of process noise in time differentiation expression of angular velocity z, noise for d_wz = 0 | 0.2           |
| proc_stddev_yaw_c      | double | Standard deviation of process noise in time differentiation expression of yaw, noise for d_yaw = omega           | 0.005         |
| proc_stddev_yaw_bias_c | double | Standard deviation of process noise in time differentiation expression of yaw_bias, noise for d_yaw_bias = 0     | 0.001         |

note: process noise for position x & y are calculated automatically from nonlinear dynamics.

## How to turn EKF parameters

### 0. Preliminaries

- Check header time in pose and twist message is set to sensor time appropriately, because time delay is calculated from this value. If it is difficult to set appropriate time due to timer synchronization problem, use `twist_additional_delay` and `pose_additional_delay` to correct the time.
- Check the relation between measurement pose and twist is appropriate (whether the derivative of pose has similar value to twist). This discrepancy is caused mainly by unit error (such as confusing radian/degree) or bias noise, and it causes large estimation errors.

### 1. Set sensor parameters

Set sensor-rate and standard-deviation from the basic information of the sensor. The `pose_measure_uncertainty_time` is for uncertainty of the header timestamp data.

- `pose_measure_uncertainty_time`
- `pose_rate`
- `twist_rate`

### 2. Set process model parameters

- `proc_stddev_vx_c` : set to maximum linear acceleration
- `proc_stddev_wz_c` : set to maximum angular acceleration
- `proc_stddev_yaw_c` : This parameter describes the correlation between the yaw and yaw-rate. Large value means the change in yaw does not correlate to the estimated yaw-rate. If this is set to 0, it means the change in estimate yaw is equal to yaw-rate. Usually this should be set to 0.
- `proc_stddev_yaw_bias_c` : This parameter is the standard deviation for the rate of change in yaw bias. In most cases, yaw bias is constant, so it can be very small, but must be non-zero.

## Kalman Filter Model

### kinematics model in update function

<img src="./media/ekf_dynamics.png" width="320">

where `b_k` is the yaw-bias.

### time delay model

The measurement time delay is handled by an augmented states [1] (See, Section 7.3 FIXED-LAG SMOOTHING).

<img src="./media/delay_model_eq.png" width="320">

Note that, although the dimension gets larger, since the analytical expansion can be applied based on the specific structures of the augmented states, the computational complexity does not significantly change.

## Test Result with Autoware NDT

<p align="center">
<img src="./media/ekf_autoware_res.png" width="600">
</p>

## reference

[1] Anderson, B. D. O., & Moore, J. B. (1979). Optimal filtering. Englewood Cliffs, NJ: Prentice-Hall.
