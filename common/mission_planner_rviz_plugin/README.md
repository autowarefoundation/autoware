# mission_planner_rviz_plugin

## MrmGoalTool

This is a copy of `rviz_default_plugins::tools::GoalTool`. Used together with the RouteSelectorPanel to set the MRM route.
After adding the tool, change the topic name to `/rviz/route_selector/mrm/goal` from the topic property panel in rviz.

## RouteSelectorPanel

This panel shows the main and mrm route state in the route_selector and the route states in the mission_planner.
Additionally, it provides clear and set functions for each main route and mrm route.

| Trigger                                | Action                                                                   |
| -------------------------------------- | ------------------------------------------------------------------------ |
| main route clear button                | call `/planning/mission_planning/route_selector/main/clear_route`        |
| mrm route clear button                 | call `/planning/mission_planning/route_selector/mrm/clear_route`         |
| `/rviz/route_selector/main/goal` topic | call `/planning/mission_planning/route_selector/main/set_waypoint_route` |
| `/rviz/route_selector/mrm/goal` topic  | call `/planning/mission_planning/route_selector/mrm/set_waypoint_route`  |
