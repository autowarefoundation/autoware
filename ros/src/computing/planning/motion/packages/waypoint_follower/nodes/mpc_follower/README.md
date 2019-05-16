# Node description

## Overview
Waypoints follower based on model predictive control (MPC) for accurate path tracking.

There are 2 nodes related to MPC path follower.
 - `/mpc_waypoint_converter` : converts `/final_waypoints` to `/mpc_waypoints` which includes waypoints behind the self position. This is to solve temporary conflict of planning system and mpc, and may be removed in a future release.
 - `/mpc_follower` : generates control command (`/twist_raw` or/and `/ctrl_cmd`) to follow `/mpc_waypoints`.

The `/mpc_waypoints_converter` is to solve temporary conflict, and will be removed in the future.

Simulation video [youtube](https://www.youtube.com/watch?v=4IO1zxsY4wU&t=18s) : comparison of pure_pursuit and mpc_follower with gazebo simulation. 



## Input and Output for mpc_follower
- input
    - /mpc_waypoints : reference waypoints (generated in mpc_waypoints_converter)
    - /current_pose : self pose
    - /vehicle_status : vehicle information (as velocity and steering angle source)
- output
    - /twist_raw : command for vehicle
    - /ctrl_cmd : command for vehicle


## node graph

<img src="./media/mpc_rqt_graph.png">


# Parameter description

## Notation
The default parameters are adjusted to the Autonomoustuff Lexus RX 450h with under 40 km/h.


## overall 

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|show_debug_info|bool|display debug info|false|
|ctrl_period|double|control period [s]|0.1|
|traj_resample_dist|double|length of resampled waypoint [m]|0.1|
|enable_path_smoothing|bool|path smoothing flug. should be true when uses path resampling to reduce resampling noise.|true|
|enable_yaw_recalculation|bool|recalculate yaw angle after resampling. Set true if yaw in received waypoints is noisy.|false|
|path_filter_moving_ave_num|int|for path smoothing filter, moving average number|35|
|path_smoothing_times|int|Number of times tof applying path smoothing filter|1|
|curvature_smoothing_num|double|curvature is calculated from three points: p(i-num), p(i), p(i+num). larger num makes less noisy values.|35|
|steering_lpf_cutoff_hz|double| cutoff frequency [hz] of butterworth-2D lowpass filter for steering output command |2.0|
|admisible_position_error|double| stop vehicle when following position error is larger than this value [m].|5.0|
|admisible_yaw_error_deg|double|stop vehicle when following yaw angle error is larger than this value [deg].|90.0|

## mpc algorithm 

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|qp_solver_type|string|QP solver option. described below in detail.|unconstraint_fast|
|vehicle_model_type|string|vehicle model option. described below in detail.|kinematics|
|prediction_holizon|int|total prediction step for MPC|50|
|prediction_sampling_time|double|prediction period for one step [s] for MPC|0.1|
|weight_lat_error|double|weight for lateral error for MPC|0.1|
|weight_heading_error|double|weight for heading error for MPC|0.0|
|weight_heading_error_squared_vel_coeff|double|weight for heading error * velocity for MPC|10.0|
|weight_steering_input|double|weight for steering error (steer command - reference steer) for MPC|1.0|
|weight_steering_input_squared_vel_coeff|double|weight for steering error (steer command - reference steer) * velocity for MPC|0.25|
|weight_lat_jerk|double|weight for lateral jerk (steer(i) - steer(i-1)) * velocity for MPC|0.0|
|weight_terminal_lat_error|double|terminal cost weight of lateral error|1.0|
|weight_terminal_heading_error|double|terminal cost weight of heading error|0.0|
|zero_ff_steer_deg|double|feedforward angle is set to zero when it is smaller than this value, for reference path noise|2.0|

## vehicle

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|wheelbase|double|wheel base length [m] for vehicle model|2.9|
|steering_gear_ratio|double| gear ratio between steering and steering wheel. If `/vehicle_status`'s angle is steering angle, set this value to 1|20.0|
|steering_tau|double|steering dynamics time constant (1d approzimation) [s] for vehicle model|0.3|
|steer_lim_deg|double|steering angle limit [deg] for vehicle model. This is also used for QP constraint.|35.0|

## QP solver type

currently, the options are
- unconstraint : use least square method to solve unconstraint QP with eigen.
- unconstraint_fast : same as above. Faster, but lower accuracy for optimization.
<!-- - qpoases_hotstart : use QPOASES with constrainted QP. -->

## vehicle model type

- kinematics : kinematics model with steering 1d-order delay.
- kinematics_no_delay : kinematics model without steering delay.


# how to tune mpc parameters

1. Set appropriate vehicle kinematics parameters of `wheelbase`, `steering_gear_ratio`, and `steer_lim_deg`. These are used to calculate feedforward command which enables following with low feedback gain.

2. Set appropriate vehicle dynamics parameters of `steering_tau`, which is aproximating delay from steering angle command to actual steering angle.

3. Set `weight_steering_input = 1.0`, `weight_lat_error = 0.1`, and others to 0.

4. Turn other weight. One of the simple way for turning is to increase `weight_lat_error` until oscillation occurs. If the vehicle is unstable with very small `weight_lat_error`, increase terminal weight :  `weight_terminal_lat_error` and `weight_terminal_heading_error`.
 Larger `prediction_holizon` and smaller `prediction_sampling_time` is also effective for stability, but it is a trade-off between computational costs.
Other parameters can be adjusted like below.
 - weight_lat_error: Reduce lateral tracking error. This acts like P gain in PID.
 - weight_heading_error: Make a drive straight. This acts like D gain in PID.
 - weight_heading_error_squared_vel_coeff : Make a drive straight in high speed range.
 - weight_steering_input: Reduce oscillation of tracking.
 - weight_steering_input_squared_vel_coeff: Reduce oscillation of tracking in high speed range.
 - weight_lat_jerk: Reduce lateral jerk.
 - weight_terminal_lat_error: Endpoint weight of MPC holizon. 
 - weight_terminal_heading_error: Endpoint weight of MPC holizon.

