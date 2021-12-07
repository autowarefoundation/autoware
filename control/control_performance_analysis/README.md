# control_performance_analysis

## Purpose

`control_performance_analysis` is the package to analyze the tracking performance of a control module.

This package is used as a tool to quantify the results of the control module.
That's why it doesn't interfere with the core logic of autonomous driving.

Based on the various input from planning, control, and vehicle, it publishes the result of analysis as `control_performance_analysis::msg::ErrorStamped` defined in this package.

## Input / Output

### Input topics

| Name                                               | Type                                                     | Description                                         |
| -------------------------------------------------- | -------------------------------------------------------- | --------------------------------------------------- |
| `/planning/scenario_planning/trajectory`           | autoware_auto_planning_msgs::msg::Trajectory             | Output trajectory from planning module.             |
| `/control/trajectory_follower/lateral/control_cmd` | autoware_auto_control_msgs::msg::AckermannLateralCommand | Output lateral control command from control module. |
| `/vehicle/status/steering_status`                  | autoware_auto_vehicle_msgs::msg::SteeringReport          | Steering information from vehicle.                  |
| `/localization/kinematic_state`                    | nav_msgs::msg::Odometry                                  | Use twist from odometry.                            |
| `/tf`                                              | tf2_msgs::msg::TFMessage                                 | Extract ego pose from tf.                           |

### Output topics

| Name                                    | Type                                            | Description                             |
| --------------------------------------- | ----------------------------------------------- | --------------------------------------- |
| `/control_performance/performance_vars` | control_performance_analysis::msg::ErrorStamped | The result of the performance analysis. |
