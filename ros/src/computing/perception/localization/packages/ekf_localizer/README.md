# Overview
This package generates robust self_pose/twist integrating pose and twist messages with vehicle model.

Some other features include
 - time delay compensation : for sensor delay, predict current position from vehicle dynamics. The delay is usually calculated by comparing the header time stamp with the current time, but additional delay time can be set with the parameter.
 - mahalanobis gate : for sensor outlier detection, Mahalanobis distance is used.
 - yaw angle compensation : yaw angle bias is estiamted, which is caused by sensor mounting error.


# Input and Output
- input
    - geometry_msgs::PoseStamped ego_pose (default: `/ndt_pose`)
    - geometry_msgs::TwistStamped ego_twist (default: `/can_twist`))
    
- output
    - tf base_link : broadcasted with *tf_rate* [Hz]
    - estimated pose (geometry_msgs::PoseStamped) : filtered pose
    - estimated twist (geometry_msgs::TwistStamped) : filtered twist


# Functions


## in timer callback with *predict_frequency*
Calculate *predict* and *update* at constant intervals in timer callback. The purpose of perform *update* at timer callback is to avoid large position (or velocity) changes due to low period measurement, such as NDT matching. If the callback period is faster than the sensor period, the same sensor value is used for update. Note that since the algorithm can deal with sensor time delay, there is no lack of consistency in using the same sensor data. 


## measurement data callback

The subscribed data is saved and used for *update* of kalman filter in timer callback. The Mahalanobis distance is calculated for the received data, and it is removed as an outlier if the value is over limit.


# Parameter description

they are set in `launch/ekf_localizer.launch` 


## for Node

|Name|Type|Description|
|:---|:---|:---|
|show_debug_info|bool|display debug info|
|predict_frequency|double|filtering and publish frequency [Hz]|
|tf_rate|double|frqcuency for tf broadcasting [Hz]|
|extend_state_step|int|extend state dimension [-]: The maximum sensor delay that can be taken into account is (extend_state_step / predict_frequency) seconds. Larger values require more computation time |
|enable_yaw_bias_estimation| bool |enable yaw bias estimation for LiDAR mount error|

## for pose measurement

|Name|Type|Description|
|:---|:---|:---|
|pose_additional_delay|double|Additional delay time for pose measurement [s]|
|pose_measure_uncertainty_time|double|Used for covariance calculation [s]|
|pose_rate|double|used for covariance calculation [Hz]|
|pose_gate_dist|double|limit of Mahalanobis distance used for outliers detection|
|pose_stddev_x|double|standard deviation for pose position x [m]|
|pose_stddev_y|double|standard deviation for pose position y [m]|
|pose_stddev_yaw|double|standard deviation for pose yaw angle [rad]|

## for twist measurement
|Name|Type|Description|
|:---|:---|:---|
|twist_additional_delay|double|Additional delay time for twist [s]|
|twist_rate|double|used for covariance calculation [Hz]|
|twist_gate_dist|double|limit of Mahalanobis distance used for outliers detection|
|twist_stddev_vx|double|standard deviation for twist linear x [m/s] |
|twist_stddev_wz|double|standard deviation for twist angular z [rad/s] |

## for process noise
|Name|Type|Description|
|:---|:---|:---|
|stddev_proc_yaw_c|double|standard deviation of process noise in time differentiation expression of yaw, noise for d_yaw = omege |
|stddev_proc_yaw_bias_c|double|standard deviation of process noise in time differentiation expression of yaw_bias, noise for d_yaw_bias = 0|
|stddev_proc_vx_c|double|standard deviation of process noise in time differentiation expression of linear velocity x, noise for d_vx = 0|
|stddev_proc_wz_c|double|standard deviation of process noise in time differentiation expression of angular velocity z, noise for d_wz = 0|

note: process noise for position x & y are calculated automatically dealing with nonlinear equation.

# kalman filter model

## kinematics model in update function
<img src="./fig/kinematics_model_eq.png" width="200">

## time delay model
<img src="./fig/delay_model_eq.png" width="320">
