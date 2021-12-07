# Mission Planner

## Purpose

`Mission Planner` calculates a route that navigates from the current ego pose to the goal pose following the given check points.
The route is made of a sequence of lanes on a static map.
Dynamic objects (e.g. pedestrians and other vehicles) and dynamic map information (e.g. road construction which blocks some lanes) are not considered during route planning.
Therefore, the output topic is only published when the goal pose or check points are given and will be latched until the new goal pose or check points are given.

The core implementation does not depend on a map format.
In current Autoware.universe, only Lanelet2 map format is supported.

## Inputs / Outputs

### input

| Name                 | Type                                 | Description                                |
| -------------------- | ------------------------------------ | ------------------------------------------ |
| `~input/vector_map`  | autoware_auto_mapping_msgs/HADMapBin | vector map of Lanelet2                     |
| `~input/goal_pose`   | geometry_msgs/PoseStamped            | goal pose                                  |
| `~input/checkpoints` | geometry_msgs/PoseStamped            | checkpoint to follow while heading to goal |

### output

| Name            | Type                                    | Description                 |
| --------------- | --------------------------------------- | --------------------------- |
| `~output/route` | autoware_auto_planning_msgs/HADMapRoute | route from ego pose to goal |

`autoware_planning_msgs/Route` consists of route sections and goal pose.

![route_sections](./media/route_sections.svg)

Route section, whose type is `autoware_auto_mapping_msgs/HADMapSegment`, is a "slice" of a road that bundles lane changeable lanes.
Note that the most atomic unit of route is `autoware_auto_mapping_msgs/MapPrimitive`, which has the unique id of a lane in a vector map and its type.
Therefore, route message does not contain geometric information about the lane since we did not want to have planning module’s message to have dependency on map data structure.

The ROS message of route section contains following three elements for each route section.

- `preferred_primitive_id`: Preferred lane to follow towards the goal.
- `primitives`: All neighbor lanes in the same direction including the preferred lane.

## Implementation

### Mission Planner

Two callbacks (goal and check points) are a trigger for route planning.
Routing graph, which plans route in Lanelet2, must be created before those callbacks, and this routing graph is created in vector map callback.

`plan route` is explained in detail in the following section.

```plantuml
@startuml
title goal callback
start

:clear previously memorized check points;

:memorize ego and goal pose as check points;

if (routing graph is ready?) then (yes)
else (no)
  stop
endif

:plan route;

:publish route;

stop
@enduml
```

Note that during the goal callback, previously memorized check points are removed, and only current ego pose and goal pose are memorized as check points.

```plantuml
@startuml
title check point callback
start

if (size of check points >= 2?) then (yes)
else (no)
  stop
endif

:memorize check point;

:plan route;

:publish route;

stop
@enduml
```

Note that at least two check points must be already memorized, which are start and goal pose, before the check point callback.

### Route Planner

`plan route` is executed with check points including current ego pose and goal pose.

```plantuml
@startuml
title plan route
start

if (goal is valid?) then (yes)
else (no)
  stop
endif

:plan path between each check points;

:initialize route lanelets;

:get preferred lanelets;

:create route sections;

if (planed route is looped?) then (no)
else (yes)
  :warn that looped route is not supported;
endif

:return route;

stop
@enduml
```

`plan path between each check points` firstly calculates closest lanes to start and goal pose.
Then routing graph of Lanelet2 plans the shortest path from start and goal pose.

`initialize route lanelets` initializes route handler, and calculates `route_lanelets`.
`route_lanelets`, all of which will be registered in route sections, are lanelets next to the lanelets in the planned path, and used when planning lane change.
To calculate `route_lanelets`,

1. All the neighbor (right and left) lanes for the planned path which is lane-changeable is memorized as `route_lanelets`.
2. All the neighbor (right and left) lanes for the planned path which is not lane-changeable is memorized as `candidate_lanelets`.
3. If the following and previous lanelets of each `candidate_lanelets` are `route_lanelets`, the `candidate_lanelet` is registered as `route_lanelets`
   - This is because even though `candidate_lanelet` (an adjacent lane) is not lane-changeable, we can pass the `candidate_lanelet` without lane change if the following and previous lanelets of the `candidate_lanelet` are `route_lanelets`

`get preferred lanelets` extracts `preferred_primitive_id` from `route_lanelets` with the route handler.

`create route sections` extracts `primitives` from `route_lanelets` for each route section with the route handler, and creates route sections.

## Limitations

- Dynamic objects (e.g. pedestrians and other vehicles) and dynamic map information (e.g. road construction which blocks some lanes) are not considered during route planning.
- Looped route is not supported.
