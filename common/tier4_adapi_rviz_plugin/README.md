# tier4_adapi_rviz_plugin

## RoutePanel

To use the panel, set the topic name from 2D Goal Pose Tool to `/rviz/routing/pose`.
By default, when a tool publish a pose, the panel immediately sets a route with that as the goal.
Enable or disable of allow_goal_modification option can be set with the check box.

Push the mode button in the waypoint to enter waypoint mode. In this mode, the pose is added to waypoints.
Press the apply button to set the route using the saved waypoints (the last one is a goal).
Reset the saved waypoints with the reset button.
