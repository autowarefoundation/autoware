# OpenPlanner - Global Planner 

## op_global_planner 

This node generate global path from start point to target point(s) on a map. Planning cost is distance only. the algorithm could be extended to handle other costs such as turning right and left busy lanes in the dynamic map. it supports autoware vector map, and special designed .kml maps.

### Outputs
Global path from start to goal, if multiple goals are set, replanning is automatic when the vehicle reaches the end one goal.


### Options
Lane change is avilable (parralel lanes are detected automatically) 
Start/Goal(s) are set from Rviz, and saved to .csv files, if rviz param is disables, start/goal(s) will be loaded from .csv file at.

### Requirements

1. vector map

### How to launch

* From a sourced terminal:

`roslaunch op_global_planner op_global_planner.launch`

* From Runtime Manager:

Computing Tab -> Mission Planning -> OpenPlanner - Global Planner  -> op_global_planner

### Subscriptions/Publications


```
Publications: 
 * /lane_waypoints_array [autoware_msgs::LaneArray]
 * /global_waypoints_rviz [visualization_msgs::MarkerArray]
 * /op_destinations_rviz [visualization_msgs::MarkerArray]
 * /vector_map_center_lines_rviz [visualization_msgs::MarkerArray]

Subscriptions: 
 * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
 * /move_base_simple/goal [geometry_msgs::PoseStamped]
 * /current_pose [geometry_msgs::PoseStamped]
 * /current_velocity [geometry_msgs::TwistStamped]
 * /vector_map_info/* 
```

![Demo Movie](https://youtu.be/BS5nLtBsXPE)
