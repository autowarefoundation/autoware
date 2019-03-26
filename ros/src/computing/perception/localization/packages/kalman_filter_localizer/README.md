# Overview
kalman filter for pose and twist estimation.

Integrate multiple measurement values and calculate self position/speed considering vehicle dynamics.

Some other features include
 - time delay compensation : for sensor delay, predict current position from vehicle dynamics. The delay is usually calculated by comparing the header time stamp with the current time, but additional delay time can be set with the parameter.
 - mahalanobis gate : for sensor outlier detection, Mahalanobis distance is used.
 - yaw angle compensation : yaw angle bias is estiamted, which is caused by sensor mounting error.


# Input and Output
- input
    - NDT pose (geometry_msgs::PoseStamped) : as a pose measurement
    - CAN twist (geometry_msgs::TwistStamped) : as a twist measurement
    
- output
    - tf base_link : broadcasted with *tf_rate* [Hz]
    - estimated pose (geometry_msgs::PoseStamped) : filtered pose
    - estimated twist (geometry_msgs::TwistStamped) : filtered twist


# Functions


## timer callback
Calculate *predict* and *update* at constant intervals in timer callback. The purpose of perform *update* at timer callback is to avoid large position (or velocity) changes due to low period measurement, such as NDT matching. If the callback period is faster than the sensor period, the same value is used for update, but because the algorithm can deal with sensor time, so there is no lack of consistency. 


## measurement data callback

The subscribed data is saved and used for *update* of kalman filter in timer callback. The Mahalanobis distance is calculated for the received data, and it is removed as an outlier if the value is over limit.


# Parameter description

they are set in `config/kalman_filter_localizer.config` 


## for Node

|Name|Type|Description|
|:---|:---|:---|
|show_debug_info|bool|display debug info|
|predict_frequency|double|filtering and publish frequency [Hz]|
|tf_rate|double|frqcuency for tf broadcasting [Hz]|
|extend_state_step|int|extend state dimension [-]: The maximum sensor delay that can be taken into account is (extend_state_step / predict_frequency) seconds. Larger values require more computation time |
|enable_yaw_bias_estimation| bool |enable yaw bias estimation for LiDAR mount error|

## for NDT measurement

|Name|Type|Description|
|:---|:---|:---|
|ndt_additional_delay|double|Additional delay time for NDT [s]|
|ndt_measure_uncertainty_time|double|Used for covariance calculation [s]|
|ndt_rate|double|used for covariance calculation [Hz]|
|ndt_gate_dist|double|limit of Mahalanobis distance used for outliers detection|
|ndt_stddev_x|double|standard deviation for ndt position x [m]|
|ndt_stddev_y|double|standard deviation for ndt position y [m]|
|ndt_stddev_yaw|double|standard deviation for ndt yaw angle [rad]|

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
|stddev_proc_yaw_c|double|standard deviation of process noise in time differentiation expression of yaw |
|stddev_proc_yaw_bias_c|double|standard deviation of process noise in time differentiation expression of yaw_bias|
|stddev_proc_vx_c|double|standard deviation of process noise in time differentiation expression of linear velocity x|
|stddev_proc_wz_c|double|standard deviation of process noise in time differentiation expression of angular velocity z|

note: process noise for position is calculated automatically from above values.

# Add new measurement

1. create subscriber and related callback function to save measured data.
2. create measurementUpdateXXX() function, which should include *time delay calculate*, *gate*, and *kalman-filter update*.
3. call measurementUpdateXXX() in timerCallback() function.

# kalman filter model

## kinematics model in update function
<img src="./fig/kinematics_model_eq.png" width="200">

## time delay model
<img src="./fig/delay_model_eq.png" width="320">
