# Simple Trajectory Follower

## Purpose

Provide a base trajectory follower code that is simple and flexible to use. This node calculates control command based on a reference trajectory and an ego vehicle kinematics.

## Design

### Inputs / Outputs

Inputs

- `input/reference_trajectory` [autoware_planning_msgs::msg::Trajectory] : reference trajectory to follow.
- `input/current_kinematic_state` [nav_msgs::msg::Odometry] : current state of the vehicle (position, velocity, etc).
- Output
- `output/control_cmd` [autoware_control_msgs::msg::Control] : generated control command.

### Parameters

| Name                    | Type  | Description                                                                                                        | Default value |
| :---------------------- | :---- | :----------------------------------------------------------------------------------------------------------------- | :------------ |
| use_external_target_vel | bool  | use external target velocity defined by parameter when true, else follow the velocity on target trajectory points. | false         |
| external_target_vel     | float | target velocity used when `use_external_target_vel` is true.                                                       | 0.0           |
| lateral_deviation       | float | target lateral deviation when following.                                                                           | 0.0           |
