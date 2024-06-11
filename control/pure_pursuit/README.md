# Pure Pursuit Controller

The Pure Pursuit Controller module calculates the steering angle for tracking a desired trajectory using the pure pursuit algorithm. This is used as a lateral controller plugin in the `autoware_trajectory_follower_node`.

## Inputs

Set the following from the [controller_node](../autoware_trajectory_follower_node/README.md)

- `autoware_planning_msgs/Trajectory` : reference trajectory to follow.
- `nav_msgs/Odometry`: current ego pose and velocity information

## Outputs

Return LateralOutput which contains the following to the controller node

- `autoware_control_msgs/Lateral`: target steering angle
- LateralSyncData
  - steer angle convergence
- `autoware_planning_msgs/Trajectory`: predicted path for ego vehicle
