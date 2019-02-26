# Overview
Waypoints follower based on model predictive control (MPC) 

# Notation
For now, Operation is limited to the following conditions.
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


Which command to output is determined by the parameter `ctrl_cmd_interface`