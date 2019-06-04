# MPC follower description

## Overview
A waypoint follower based on model predictive control (MPC) for accurate path tracking. This can be used as a waypoint_follower, as well as other path following nodes like pure_pursuit.


There are 2 nodes related to MPC follower.
 - `/mpc_waypoint_converter` : converts `/final_waypoints` to `/mpc_waypoints` which includes waypoints behind the self position. This is to solve temporary conflict of planning system and mpc follower so that mpc follower can be used in the same way as pure_pursuit. This will be removed in a future release.
 - `/mpc_follower` : generates control command (`/twist_raw` or/and `/ctrl_cmd`) to follow `/mpc_waypoints`.

[Video](https://www.youtube.com/watch?v=4IO1zxsY4wU&t=18s) : comparison of pure_pursuit and mpc_follower with gazebo simulation. 



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
The default parameters are adjusted to the Autonomoustuff Lexus RX 450h for under 40 km/h driving.


## overall 

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|show_debug_info|bool|display debug info|false|
|ctrl_period|double|control period [s]|0.03|
|traj_resample_dist|double|distance of waypoints in resampling [m]|0.1|
|enable_path_smoothing|bool|path smoothing flag. This should be true when uses path resampling to reduce resampling noise.|true|
|enable_yaw_recalculation|bool|recalculate yaw angle after resampling. Set true if yaw in received waypoints is noisy.|false|
|path_filter_moving_ave_num|int|number of data points moving average filter for path smoothing|35|
|path_smoothing_times|int|number of times of applying path smoothing filter|1|
|curvature_smoothing_num|double|index distance of points used in curvature calculation: p(i-num), p(i), p(i+num). larger num makes less noisy values.|35|
|steering_lpf_cutoff_hz|double| cutoff frequency of lowpass filter for steering output command [hz]|3.0|
|admisible_position_error|double| stop vehicle when following position error is larger than this value [m].|5.0|
|admisible_yaw_error_deg|double|stop vehicle when following yaw angle error is larger than this value [deg].|90.0|

## mpc algorithm 

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|qp_solver_type|string|QP solver option. described below in detail.|unconstraint_fast|
|vehicle_model_type|string|vehicle model option. described below in detail.|kinematics|
|prediction_horizon|int|total prediction step for MPC|70|
|prediction_sampling_time|double|prediction period for one step [s]|0.1|
|weight_lat_error|double|weight for lateral error|0.1|
|weight_heading_error|double|weight for heading error|0.0|
|weight_heading_error_squared_vel_coeff|double|weight for heading error * velocity|5.0|
|weight_steering_input|double|weight for steering error (steer command - reference steer)|1.0|
|weight_steering_input_squared_vel_coeff|double|weight for steering error (steer command - reference steer) * velocity|0.25|
|weight_lat_jerk|double|weight for lateral jerk (steer(i) - steer(i-1)) * velocity|0.0|
|weight_terminal_lat_error|double|terminal cost weight for lateral error|1.0|
|weight_terminal_heading_error|double|terminal cost weight for heading error|0.1|
|zero_ff_steer_deg|double|threshold of feedforward angle [deg]. feedforward angle smaller than this value is set to zero.|2.0|


## vehicle

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|wheelbase|double|wheel base length for vehicle model [m] |2.9|
|steering_gear_ratio|double| gear ratio between steering and steering wheel. If a steering wheel angle is set in `/vehicle_status`, set this value to 1|20.0|
|steering_tau|double|steering dynamics time constant (1d approximation) for vehicle model [s]|0.3|
|steer_lim_deg|double|steering angle limit for vehicle model [deg]. This is also used for QP constraint.|35.0|

## QP solver type

currently, the options are
- unconstraint : use least square method to solve unconstraint QP with eigen.
- unconstraint_fast : same as above. Faster, but lower accuracy for optimization.


## vehicle model type

- kinematics : bicycle kinematics model with steering 1st-order delay
- kinematics_no_delay : bicycle kinematics model without steering delay
- dynamics : bicycle dynamics model considering slip angle

The `kinematics` model are being used as default. Please see the reference[1] for more detail.

# how to tune MPC parameters

1. Set appropriate vehicle kinematics parameters `wheelbase`, `steering_gear_ratio`, and `steer_lim_deg`. These values give a vehicle information to the controller for path following. Errors in these values cause fundamental tracking error. Whether these values are correct can be confirmed by comparing the angular velocity obtained from the model (`/mpc_follower/debug/angvel_from_steer`) and the actual angular velocity (such as `/estimate_twist/angular/z`).

2. Set appropriate vehicle dynamics parameters of `steering_tau`, which is approximated delay from steering angle command to actual steering angle.

3. Set `weight_steering_input` = 1.0, `weight_lat_error` = 0.1, and other weights to 0. If the vehicle oscillates when driving with low speed, set `weight_lat_error` smaller.

4. Adjust other weights. One of the simple way for tuning is to increase `weight_lat_error` until oscillation occurs. If the vehicle is unstable with very small `weight_lat_error`, increase terminal weight :  `weight_terminal_lat_error` and `weight_terminal_heading_error` to improve tracking stability.
 Larger `prediction_horizon` and smaller `prediction_sampling_time` is effective for tracking performance, but it is a trade-off between computational costs.
Other parameters can be adjusted like below.
 - `weight_lat_error`: Reduce lateral tracking error. This acts like P gain in PID.
 - `weight_heading_error`: Make a drive straight. This acts like D gain in PID.
 - `weight_heading_error_squared_vel_coeff` : Make a drive straight in high speed range.
 - `weight_steering_input`: Reduce oscillation of tracking.
 - `weight_steering_input_squared_vel_coeff`: Reduce oscillation of tracking in high speed range.
 - `weight_lat_jerk`: Reduce lateral jerk.
 - `weight_terminal_lat_error`: Preferable to set a higher value than normal lateral weight `weight_lat_error` for stability.
 - `weight_terminal_heading_error`: Preferable to set a higher value than normal heading weight `weight_heading_error` for stability.

 # reference 

 [1] Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking", Robotics Institute, Carnegie Mellon University, February 2009.
