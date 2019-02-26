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

# Node Configuration

This node includes 
- path filter : apply moving average filter to reduce waypoint noise. 
- mpc solver : least square method is employed to solve the optimization problem. Constraints are not supported for now.
    - vehicle model : used to make MPC matrix. Now, bicycle kinematics model is only available.
- lowpass filter : 2nd-order butterworth filter 
