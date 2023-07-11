# Pure Pursuit Controller

The Pure Pursuit Controller module calculates the steering angle for tracking a desired trajectory using the pure pursuit algorithm. This is used as a lateral controller plugin in the `trajectory_follower_node`.

## Inputs

Set the following from the [controller_node](../trajectory_follower_node/README.md)

- `autoware_auto_planning_msgs/Trajectory` : reference trajectory to follow.
- `nav_msgs/Odometry`: current ego pose and velocity information

## Outputs

Return LateralOutput which contains the following to the controller node

- `autoware_auto_control_msgs/AckermannLateralCommand`: target steering angle
- LateralSyncData
  - steer angle convergence
- `autoware_auto_planning_msgs/Trajectory`: predicted path for ego vehicle
