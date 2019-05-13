# Overview
Waypoints follower based on model predictive control (MPC) 

# Notation
- vehicle
    - gazebo (simulation)
    - Lexus RX450h (under 40km/h)

# Input and Output
- input
    - /mpc_waypoints : reference waypoints (generated in mpc_waypoints_converter)
    - /current_pose : self pose
    - /vehicle_status : vehicle information (as velocity and steering angle source)
- output
    - /twist_raw : command for vehicle
    - /ctrl_cmd : command for vehicle



The output interface is determined by the parameter `ctrl_cmd_interface`. Default is for both.



# How to run

## gazebo example

1. run gazebo at first.

https://github.com/CPFL/Autoware/edit/develop/ros/src/simulation/gazebo_simulator/README.md


2. launch mpc_follower with simulation config.

```
$ roslaunch waypoint_follower mpc_follower_sim.launch
```

video link: 

https://www.youtube.com/watch?v=4IO1zxsY4wU&t=18s

# Nodes

- mpc_waypoints_converter
    - to generate mpc_waypoints, which includes the path behind the self position.
- mpc_follower
    - to calculate steering and velocity command to follow the reference waypoints. It includes following functions.
        - path filter : apply path smoothing filter to reduce waypoint noise.
        - mpc solver : solve the optimization problem of MPC.
        - lowpass filter : applied to the mpc output: 2nd-order butterworth filter 


# Parameter description

## overall 

|Name|Type|Description|
|:---|:---|:---|
|show_debug_info|bool|display debug info|
|ctrl_period|double|control period [s]|
|traj_resample_dist|double|length for resampling trajectory [m]|
|enable_path_smoothing|bool|path smoothing flug. should be true when uses path resampling to reduce resampling noise.|
|enable_yaw_recalculation|bool|recalculate yaw angle after resampling. Set true if yaw in received waypoints is noisy.|
|path_filter_moving_ave_num|double|for path smoothing filter, moving average number|
|path_smoothing_times|int|Number of times tof applying path smoothing filter|
|curvature_smoothing_num|double|curvature is calculated from three points: p(i-num), p(i), p(i+num). larger num makes less noisy values.|
|steering_lpf_cutoff_hz|double| cutoff frequency [hz] of butterworth-2D lowpass filter for steering output command |
|admisible_position_error|double| stop vehicle when following position error is larger than this value [m].|
|admisible_yaw_error_deg|double|stop vehicle when following yaw angle error is larger than this value [deg].|

## mpc algorithm 

|Name|Type|Description|
|:---|:---|:---|
|qp_solver_type|string|QP solver option. descrived below in detail.|
|vehicle_model_type|string|vehicle model option. descrived below in detail.|
|mpc_n|double|total prediction step for MPC|
|mpc_dt|double|prediction period for one step [s] for MPC|
|mpc_weight_lat_error|double|weight for lateral error for MPC|
|mpc_weight_heading_error|double|weight for heading error for MPC|
|mpc_weight_heading_error_squared_vel_coeff|double|weight for heading error * velocity for MPC|
|mpc_weight_steering_input|double|weight for steering error (steer command - reference steer) for MPC|
|mpc_weight_steering_input_squared_vel_coeff|double|weight for steering error (steer command - reference steer) * velocity for MPC|
|mpc_weight_lat_jerk|double|weight for lateral jerk (steer(i) - steer(i-1)) * velocity for MPC|
|mpc_weight_endpoint_Q_scale|double|endpoint error weight to improve mpc stability|
|mpc_zero_ff_steer_deg|double|feedforward angle is set to zero when it is smaller than this value, for reference path noise|

## vehicle

|Name|Type|Description|
|:---|:---|:---|
|vehicle_model_steer_tau|double|steering dynamics time constant (1d approzimation) [s] for vehicle model|
|vehicle_model_wheelbase|double|wheel base length [m] for vehicle model|
|steer_lim_deg|double|steering angle limit [deg] for vehicle model. This is also used for QP constraint.|

## QP solver type

currently, the options are
- unconstraint : use least square method to solve unconstraint QP with eigen.
- unconstraint_fast : same as above. Faster, but lower accuracy for optimization.
- qpoases_hotstart : use QPOASES with constrainted QP.

## vehicle model type

- kinematics : kinematics model with steering 1d-order delay.
- kinematics_no_delay : kinematics model without steering delay.


