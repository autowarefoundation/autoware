# Overview
Waypoints follower based on model predictive control (MPC) 

# Notation
For now, the operation is limited to the following conditions.
- functional limits
    - no obstacle avoidance or stop (subscrive /base_waypoints directly)

- vehicle
    - gazebo
    - Lexus RX450h (under 20km/h)

# Input and Output
- input
    - /base_waypoints : reference waypoints
    - /current_pose : self pose
    - /vehicle_status : vehicle information (can velocity, steering angle)
- output
    - /twist_raw : command for vehicle
    - /ctrl_cmd : command for vehicle



Which command to output is determined by the parameter `ctrl_cmd_interface`. Default is for both.



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

# Functions

This node includes 
- path filter : apply path smoothing filter to reduce waypoint noise. For now, moving average filter is used. 
- mpc solver : solve the optimization problem of MPC. For now, the least square method is used, so constraints are not supported.
    - vehicle model : to generate MPC matrix. For now, bicycle kinematics model is only available.
- lowpass filter : 2nd-order butterworth filter 

# Parameter description

## overall 

|Name|Type|Description|
|:---|:---|:---|
|show_debug_info|bool|display debug info|
|ctrl_period|double|control period [s]|
|traj_resample_dist|double|length for resampling trajectory [m]|
|enable_path_smoothing|bool|path smoothing flug. should be true when uses path resampling to reduce resampling noise.|
|enable_yaw_recalculation|bool|recalculate yaw angle after resampling. Set true if yaw in received waypoints has noise.|
|path_filter_moving_ave_num|double|for path smoothing filter, moving average number|
|path_smoothing_times|int|Number of times tof applying path smoothing filter|
|curvature_smoothing_num|double|curvature is calculated from three points: p(i-num), p(i), p(i+num). larger num makes less noisy values.|
|steering_lpf_cutoff_hz|double| cutoff frequency [hz] of butterworth-2D lowpass filter for steering output command |
|admisible_position_error|double| stop vehicle when following position error is larger than this value [m].|
|admisible_yaw_error_deg|double|stop vehicle when following yaw angle error is larger than this value [deg].|

## mpc algorithm 

|Name|Type|Description|
|:---|:---|:---|
|mpc_n|double|total prediction step for MPC|
|mpc_dt|double|prediction period for one step [s] for MPC|
|mpc_weight_lat_error|double|weight for lateral error for MPC|
|mpc_weight_heading_error|double|weight for heading error for MPC|
|mpc_weight_steering_input|double|weight for steering error (actual steer - reference steer) for MPC|
|mpc_weight_steering_input_vel_coeff|double|velocity coefficient of weight for steering error (actual steer - reference steer) for MPC|
|mpc_delay_compensation_time|double |time delay compensation for MPC<br>  (Since this is effective under severe assumptions, large values are not acceptable. Set smaller than 0.05s)|
|mpc_zero_curvature_range|double|reference curvature is set to zeto when it is smaller than this valuse for noise reduction for MPC|

## vehicle model

|Name|Type|Description|
|:---|:---|:---|
|vehicle_model_steer_tau|double|steering dynamics time constant (1d approzimation) [s] for vehicle model|
|vehicle_model_wheelbase|double|wheel base length [m] for vehicle model|
|vehicle_model_steer_lim_deg|double|steering angle limit [deg] for vehicle model|