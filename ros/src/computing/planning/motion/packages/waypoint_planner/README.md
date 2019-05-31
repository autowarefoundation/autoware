# Waypoint Planner

Waypoint planner package provides the local planner nodes that dynamically plan avoidance behavior, velocity on waypoints, and so on.

## Waypoint Planner - Astar Avoid

`astar_avoid` node has two mode, Relay mode and Avoidance mode. You can switch these modes by `enable_avoidance` parameter.

- Relay mode: Not avoid planning and just publishing waypoints from self pose.
- Avoidance mode: Avoiding obstacles by Hybrid-A* search algorithm in `astar_search` package with internal state transition

*NOTE* : If you have `wayarea` in your ADAS map, it's possible to limit search area and realize more safety planning by enabling `Use Wayarea` in `costmap_generator` node. Please see the results in below demo videos.

Please see also: mission/packages/freespace_planner/README.md

### How to launch

* From Runtime Manager:

Computing -> Motion Planning -> waypoint_planner -> astar_avoid

* From CLI:

`$ roslaunch waypoint_planner astar_avoid.launch`

### Parameters

Parameters can be set in both Launch file and Runtime manager:

| Parameter in RM | Parameter in Launch | Type | Description | Default |
| --- | --- | --- | --- | --- |
| `Enable Avoidance` | `enable_avoidance` | *Bool* | Enable avoidance mode | `false` |
| `Costmap Topic` | `costmap_topic` | *String* | Costmap topic for Hybrid-A* search | `semantics/costmap_generator/occupancy_grid` |
| `Waypoint Velocity` | `avoid_waypoints_velocity` | *Double* | Constant velocity on planned waypoints [km/h] | `10.0` |
| `Avoidance Start Velocity` | `avoid_start_velocity` | *Double* | Self velocity for staring avoidance behavior [km/h] | `5.0` |
| `Replan Interval` | `replan_interval` | *Double* | Replan interval for avoidance planning [Hz] | `2.0` |
| - | `safety_waypoints_size` | *Int* | Output waypoint size [-] | `100` |
| - | `update_rate` | *Double* | Publishing rate [Hz] | `10.0` |
| - | `search_waypoints_size` | *Int* | Range of waypoints for incremental search [-] | `50` |
| - | `search_waypoints_delta` | *Int* | Skipped waypoints for incremental search [-] | `2` |

### Subscriptions/Publications

```
Node [/astar_avoid]
Publications:
 * /safety_waypoints [autoware_msgs/Lane]

Subscriptions:
 * /base_waypoints [autoware_msgs/Lane]
 * /closest_waypoint [std_msgs/Int32]
 * /current_pose [geometry_msgs/PoseStamped]
 * /current_velocity [geometry_msgs/TwistStamped]
 * /semantics/costmap_generator/occupancy_grid [nav_msgs/OccupancyGrid]
 * /obstacle_waypoint [std_msgs/Int32]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]
```

### Demo videos

#### Dynamically avoiding (senario 1)
[![Hybrid A*, dynamically avoiding (scenario 1)](https://img.youtube.com/vi/o_WXfPh9JKA/sddefault.jpg)](https://youtu.be/o_WXfPh9JKA)

#### Dynamically avoiding (senario 2)
[![Hybrid A*, dynamically avoiding (scenario 12](https://img.youtube.com/vi/fqXIlWuMuDk/sddefault.jpg)](https://youtu.be/fqXIlWuMuDk)

#### Statically avoiding by big re-routing
[![Hybrid A*, statically avoiding by big re-routing](https://img.youtube.com/vi/J-3J-EiBP38/sddefault.jpg)](https://youtu.be/J-3J-EiBP38)

## Waypoint Planner - Velocity Set

// TODO
